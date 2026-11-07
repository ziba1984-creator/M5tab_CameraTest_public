#include <dirent.h>
#include <inttypes.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include "driver/jpeg_encode.h"
#include "esp_check.h"
#include "esp_codec_dev.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "bsp/m5stack_tab5.h"
#include "app_config.h"
#include "avi_writer.h"
#include "cam_builtin.h"
#include "recorder.h"

static const char *TAG = "recorder";

#define AUDIO_CHANNELS  1
#define AUDIO_CHUNK     (APP_REC_AUDIO_RATE * APP_REC_AUDIO_CHUNK_MS / 1000 * 2 * AUDIO_CHANNELS)
#define VIDEO_SLOT      (APP_REC_VIDEO_SLOT_KB * 1024)
#define EVT_FRAME       BIT0

typedef enum { ITEM_VIDEO, ITEM_AUDIO, ITEM_STOP } item_kind_t;

typedef struct {
    uint8_t kind;
    uint8_t slot;
    uint32_t len;
    int64_t t_us;       /* capture time (video) */
} item_t;

/* Encoded frames and audio chunks wait in fixed slots; the free queues hold
 * the slot numbers not in use, the write queue the filled ones in order. */
static uint8_t *s_vslot[APP_REC_VIDEO_SLOTS];
static uint8_t *s_aslot[APP_REC_AUDIO_SLOTS];
static QueueHandle_t s_vfree, s_afree, s_writeq;
static SemaphoreHandle_t s_audio_go, s_stopped;
static EventGroupHandle_t s_evt;
static jpeg_encoder_handle_t s_enc;
static esp_codec_dev_handle_t s_mic;
static void *s_io_buf;

static bool s_ready;                    /* recorder_init() completed */
static atomic_bool s_active;            /* sources feed the write queue */
static bool s_card;                     /* microSD mounted (UI task only) */
static uint32_t s_next_no;              /* next file number (UI task, then writer) */

/* Owned by the writer task while recording. */
static avi_writer_t *s_avi;
static int64_t s_seg_first_us, s_seg_last_us;

/* Written by the tasks without locking; readers only need approximate values. */
static recorder_stats_t s_stats;
static int64_t s_start_us;

/* ---- card and files ---------------------------------------------------------- */

static bool mount_card(void)
{
    const esp_vfs_fat_sdmmc_mount_config_t mount = {
        .format_if_mount_failed = false,    /* never erase a card the user put in */
        .max_files = 2,
        .allocation_unit_size = 32 * 1024,
    };
    bsp_sdcard_cfg_t cfg = { .mount = &mount };
    esp_err_t err = bsp_sdcard_sdmmc_mount(&cfg);
    if (err != ESP_OK) {
        bsp_sdcard_unmount();   /* release the LDO channel so a later retry can claim it */
        ESP_LOGW(TAG, "microSD not usable (%s); FAT32 card needed", esp_err_to_name(err));
        return false;
    }
    mkdir(APP_REC_DIR, 0755);
    uint64_t total = 0, free_bytes = 0;
    esp_vfs_fat_info(BSP_SD_MOUNT_POINT, &total, &free_bytes);
    ESP_LOGI(TAG, "microSD mounted: %" PRIu64 " MB free of %" PRIu64 " MB",
             free_bytes / (1024 * 1024), total / (1024 * 1024));
    return true;
}

/* Rnnnn.AVI: continue after the highest number already on the card. */
static uint32_t next_file_no(void)
{
    uint32_t max = 0;
    DIR *dir = opendir(APP_REC_DIR);
    if (dir) {
        struct dirent *e;
        while ((e = readdir(dir)) != NULL) {
            unsigned n;
            if (sscanf(e->d_name, "R%4u.AVI", &n) == 1 && n > max) {
                max = n;
            }
        }
        closedir(dir);
    }
    return max + 1;
}

static esp_err_t open_file(uint32_t no)
{
    char path[48];
    snprintf(path, sizeof(path), APP_REC_DIR "/R%04" PRIu32 ".AVI", no);
    const avi_config_t cfg = {
        .width = APP_CAM_BUILTIN_W,
        .height = APP_CAM_BUILTIN_H,
        .audio_rate = APP_REC_AUDIO_RATE,
        .audio_channels = AUDIO_CHANNELS,
        .io_buf = s_io_buf,
        .io_buf_size = APP_REC_WRITE_BUF_KB * 1024,
    };
    ESP_RETURN_ON_ERROR(avi_open(path, &cfg, &s_avi), TAG, "open %s", path);
    s_seg_first_us = s_seg_last_us = 0;
    s_stats.file_no = no;
    ESP_LOGI(TAG, "recording to %s", path);
    return ESP_OK;
}

/* Duration from the first to the last frame plus one average frame period. */
static esp_err_t close_file(void)
{
    if (!s_avi) {
        return ESP_OK;
    }
    uint32_t frames = avi_video_frames(s_avi);
    int64_t span = s_seg_last_us - s_seg_first_us;
    int64_t duration = frames > 1 ? span + span / (frames - 1) : 0;
    uint64_t bytes = avi_bytes(s_avi);
    esp_err_t err = avi_close(s_avi, duration);
    s_avi = NULL;
    ESP_LOGI(TAG, "closed R%04" PRIu32 ".AVI: %" PRIu32 " frames, %.1f s, %.1f fps, %.1f MB%s",
             s_stats.file_no, frames, duration / 1e6, duration ? frames * 1e6 / duration : 0.0,
             bytes / 1048576.0, err == ESP_OK ? "" : " (FAILED)");
    return err;
}

/* ---- tasks ------------------------------------------------------------------------- */

static void stop_sources(void)
{
    atomic_store(&s_active, false);
    cam_builtin_set_recording(false);
}

static void give_back(const item_t *it)
{
    if (it->kind == ITEM_VIDEO) {
        xQueueSend(s_vfree, &it->slot, 0);
    } else if (it->kind == ITEM_AUDIO) {
        xQueueSend(s_afree, &it->slot, 0);
    }
}

static void writer_task(void *arg)
{
    for (;;) {
        item_t it;
        xQueueReceive(s_writeq, &it, portMAX_DELAY);
        if (it.kind == ITEM_STOP) {
            close_file();
            xSemaphoreGive(s_stopped);
            continue;
        }
        if (!s_avi) {           /* stopped or failed: late items are dropped */
            give_back(&it);
            continue;
        }
        esp_err_t err = ESP_OK;
        int64_t t0 = esp_timer_get_time();
        if (it.kind == ITEM_VIDEO) {
            if (s_seg_first_us && it.t_us - s_seg_first_us >= (int64_t)APP_REC_SEGMENT_S * 1000000) {
                err = close_file();
                if (err == ESP_OK) {
                    err = open_file(++s_next_no);
                }
            }
            if (err == ESP_OK) {
                err = avi_write_video(s_avi, s_vslot[it.slot], it.len);
                if (!s_seg_first_us) {
                    s_seg_first_us = it.t_us;
                }
                s_seg_last_us = it.t_us;
                s_stats.frames++;
            }
        } else {
            err = avi_write_audio(s_avi, s_aslot[it.slot], it.len);
            s_stats.audio_chunks++;
        }
        uint32_t ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);
        if (ms > s_stats.write_ms_max) {
            s_stats.write_ms_max = ms;
        }
        give_back(&it);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "write failed: recording stopped (card full or removed?)");
            stop_sources();
            close_file();
            s_stats.state = RECORDER_ERROR;
        } else if (s_avi) {
            s_stats.bytes = avi_bytes(s_avi);
        }
    }
}

static void encoder_task(void *arg)
{
    frame_mbox_t mb = cam_builtin_rec_mbox();
    for (;;) {
        xEventGroupWaitBits(s_evt, EVT_FRAME, pdTRUE, pdFALSE, pdMS_TO_TICKS(100));
        video_frame_t f;
        if (!frame_mbox_take(mb, &f)) {
            continue;
        }
        int64_t t = esp_timer_get_time();
        uint8_t slot;
        if (!atomic_load(&s_active) || xQueueReceive(s_vfree, &slot, 0) != pdTRUE) {
            if (atomic_load(&s_active)) {
                s_stats.frames_dropped++;
            }
            frame_mbox_release(mb, &f);     /* the camera buffer goes straight back */
            continue;
        }
        jpeg_encode_cfg_t cfg = {
            .width = f.w,
            .height = f.h,
            .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
            .sub_sample = JPEG_DOWN_SAMPLING_YUV420,
            .image_quality = APP_REC_QUALITY,
        };
        uint32_t len = 0;
        esp_err_t err = jpeg_encoder_process(s_enc, &cfg, f.data, (uint32_t)f.stride * f.h * 2,
                                             s_vslot[slot], VIDEO_SLOT, &len);
        frame_mbox_release(mb, &f);
        uint32_t us = (uint32_t)(esp_timer_get_time() - t);
        if (us > s_stats.encode_us_max) {
            s_stats.encode_us_max = us;
        }
        item_t it = { .kind = ITEM_VIDEO, .slot = slot, .len = len, .t_us = t };
        if (err != ESP_OK || xQueueSend(s_writeq, &it, 0) != pdTRUE) {
            xQueueSend(s_vfree, &slot, 0);
            s_stats.frames_dropped++;
        }
    }
}

/* Read-only loop: the I2S DMA ring holds only ~0.1 s, so nothing else may run
 * here while recording. A chunk with no free slot is read into scratch and dropped. */
static void audio_task(void *arg)
{
    static uint8_t scratch[AUDIO_CHUNK];
    for (;;) {
        xSemaphoreTake(s_audio_go, portMAX_DELAY);
        esp_codec_dev_sample_info_t fs = {
            .bits_per_sample = 16,
            .channel = AUDIO_CHANNELS,
            .sample_rate = APP_REC_AUDIO_RATE,
        };
        if (esp_codec_dev_open(s_mic, &fs) != ESP_CODEC_DEV_OK) {
            ESP_LOGE(TAG, "microphone open failed: recording without sound");
            continue;
        }
        esp_codec_dev_set_in_gain(s_mic, APP_REC_AUDIO_GAIN_DB);
        for (int skipped = 0; skipped < APP_REC_AUDIO_SKIP_MS; skipped += APP_REC_AUDIO_CHUNK_MS) {
            esp_codec_dev_read(s_mic, scratch, sizeof(scratch));   /* start-up click */
        }
        while (atomic_load(&s_active)) {
            uint8_t slot;
            bool have = xQueueReceive(s_afree, &slot, 0) == pdTRUE;
            if (esp_codec_dev_read(s_mic, have ? s_aslot[slot] : scratch, AUDIO_CHUNK) != ESP_CODEC_DEV_OK) {
                ESP_LOGE(TAG, "microphone read failed");
                if (have) {
                    xQueueSend(s_afree, &slot, 0);
                }
                break;
            }
            item_t it = { .kind = ITEM_AUDIO, .slot = slot, .len = AUDIO_CHUNK };
            if (!have || xQueueSend(s_writeq, &it, 0) != pdTRUE) {
                if (have) {
                    xQueueSend(s_afree, &slot, 0);
                }
                s_stats.audio_dropped++;
            }
        }
        esp_codec_dev_close(s_mic);
    }
}

/* ---- public ------------------------------------------------------------------------ */

esp_err_t recorder_init(void)
{
    const jpeg_encode_engine_cfg_t eng = { .intr_priority = 0, .timeout_ms = 100 };
    ESP_RETURN_ON_ERROR(jpeg_new_encoder_engine(&eng, &s_enc), TAG, "JPEG encoder");
    const jpeg_encode_memory_alloc_cfg_t out_cfg = { .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER };

    s_vfree = xQueueCreate(APP_REC_VIDEO_SLOTS, sizeof(uint8_t));
    s_afree = xQueueCreate(APP_REC_AUDIO_SLOTS, sizeof(uint8_t));
    s_writeq = xQueueCreate(APP_REC_VIDEO_SLOTS + APP_REC_AUDIO_SLOTS + 2, sizeof(item_t));
    s_audio_go = xSemaphoreCreateBinary();
    s_stopped = xSemaphoreCreateBinary();
    s_evt = xEventGroupCreate();
    s_io_buf = heap_caps_malloc(APP_REC_WRITE_BUF_KB * 1024, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    ESP_RETURN_ON_FALSE(s_vfree && s_afree && s_writeq && s_audio_go && s_stopped && s_evt && s_io_buf,
                        ESP_ERR_NO_MEM, TAG, "no memory");
    for (uint8_t i = 0; i < APP_REC_VIDEO_SLOTS; i++) {
        size_t got = 0;
        s_vslot[i] = jpeg_alloc_encoder_mem(VIDEO_SLOT, &out_cfg, &got);
        ESP_RETURN_ON_FALSE(s_vslot[i] && got >= VIDEO_SLOT, ESP_ERR_NO_MEM, TAG, "no memory for video slots");
        xQueueSend(s_vfree, &i, 0);
    }
    for (uint8_t i = 0; i < APP_REC_AUDIO_SLOTS; i++) {
        s_aslot[i] = heap_caps_malloc(AUDIO_CHUNK, MALLOC_CAP_SPIRAM);
        ESP_RETURN_ON_FALSE(s_aslot[i], ESP_ERR_NO_MEM, TAG, "no memory for audio slots");
        xQueueSend(s_afree, &i, 0);
    }

    s_mic = bsp_audio_codec_microphone_init();
    if (!s_mic) {
        ESP_LOGW(TAG, "microphone unavailable: recordings will have no sound");
    }
    frame_mbox_set_notify(cam_builtin_rec_mbox(), s_evt, EVT_FRAME);
    s_card = mount_card();
    s_stats.state = s_card ? RECORDER_IDLE : RECORDER_NO_CARD;

    ESP_RETURN_ON_FALSE(xTaskCreate(writer_task, "rec_write", APP_TASK_REC_WRITE_STACK, NULL,
                                    APP_TASK_REC_WRITE_PRIO, NULL) == pdPASS &&
                        xTaskCreate(encoder_task, "rec_enc", APP_TASK_REC_ENC_STACK, NULL,
                                    APP_TASK_REC_ENC_PRIO, NULL) == pdPASS &&
                        xTaskCreate(audio_task, "rec_audio", APP_TASK_REC_AUDIO_STACK, NULL,
                                    APP_TASK_REC_AUDIO_PRIO, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task create failed");
    s_ready = true;
    ESP_LOGI(TAG, "ready: JPEG q%d, %d Hz audio, %d s per file", APP_REC_QUALITY, APP_REC_AUDIO_RATE,
             APP_REC_SEGMENT_S);
    return ESP_OK;
}

esp_err_t recorder_start(void)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (atomic_load(&s_active)) {
        return ESP_OK;
    }
    if (!s_card) {
        s_card = mount_card();
    }
    if (!s_card) {
        s_stats.state = RECORDER_NO_CARD;
        return ESP_ERR_NOT_FOUND;
    }
    s_next_no = next_file_no();
    if (open_file(s_next_no) != ESP_OK) {
        /* Card removed or full: remount on the next attempt. */
        bsp_sdcard_unmount();
        s_card = false;
        s_stats.state = RECORDER_NO_CARD;
        return ESP_FAIL;
    }
    s_stats.frames = s_stats.frames_dropped = s_stats.audio_chunks = s_stats.audio_dropped = 0;
    s_stats.bytes = 0;
    s_stats.encode_us_max = s_stats.write_ms_max = 0;
    s_start_us = esp_timer_get_time();
    s_stats.state = RECORDER_RECORDING;
    atomic_store(&s_active, true);
    if (s_mic) {
        xSemaphoreGive(s_audio_go);
    }
    cam_builtin_set_recording(true);
    return ESP_OK;
}

void recorder_stop(void)
{
    if (!atomic_load(&s_active)) {
        return;
    }
    stop_sources();
    vTaskDelay(pdMS_TO_TICKS(APP_REC_AUDIO_CHUNK_MS + 50));    /* let in-flight items reach the queue */
    const item_t stop = { .kind = ITEM_STOP };
    if (xQueueSend(s_writeq, &stop, pdMS_TO_TICKS(1000)) != pdTRUE ||
        xSemaphoreTake(s_stopped, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "the writer did not finish the file");
    }
    if (s_stats.state == RECORDER_RECORDING) {
        s_stats.state = RECORDER_IDLE;
    }
    ESP_LOGI(TAG, "stopped: %" PRIu32 " frames (%" PRIu32 " dropped), %" PRIu32 " audio chunks (%" PRIu32
                  " dropped), longest write %" PRIu32 " ms, longest encode %" PRIu32 " us",
             s_stats.frames, s_stats.frames_dropped, s_stats.audio_chunks, s_stats.audio_dropped,
             s_stats.write_ms_max, s_stats.encode_us_max);
}

void recorder_get_stats(recorder_stats_t *out)
{
    *out = s_stats;
    out->seconds = s_stats.state == RECORDER_RECORDING
                   ? (uint32_t)((esp_timer_get_time() - s_start_us) / 1000000) : 0;
}
