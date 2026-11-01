#include <stdlib.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"
#include "bsp/m5stack_tab5.h"
#include "app_config.h"
#include "cam_uvc.h"
#include "cam_uvc_ctrl.h"
#include "jpeg_dec.h"

static const char *TAG = "cam_uvc";

#define STR_(x)  #x
#define STR(x)   STR_(x)

#define INTERVAL_100NS(fps)     (10000000U / (fps))     /* UVC frame intervals are in 100 ns units */

typedef enum { EVT_CONNECTED, EVT_DISCONNECTED, EVT_POWER_CYCLE } evt_kind_t;

typedef struct {
    evt_kind_t kind;
    uint8_t addr, idx;
} mgr_evt_t;

typedef struct {
    const uvc_host_frame_t *frame;
    uint32_t gen;               /* stream generation the frame belongs to */
} rx_item_t;

static QueueHandle_t s_evt_q;           /* driver/stream callbacks -> manager task */
static QueueHandle_t s_rx_q;            /* frame callback -> decode task */
static QueueHandle_t s_out_free_q;      /* free decoded-frame buffers (indexes) */
static SemaphoreHandle_t s_lock;        /* guards s_stream / s_gen against the decode task */
static uvc_host_stream_hdl_t s_stream;
static volatile uint32_t s_gen;
static frame_mbox_t s_mbox;
static uint8_t *s_out[APP_UVC_OUT_BUFS];
static size_t s_out_cap;
static cam_uvc_stats_t s_stats;
static volatile TickType_t s_last_rx;   /* tick of the last received frame (stall watchdog) */
static int s_open_failures;             /* consecutive failed opens of a present camera */

/* ---- callbacks ------------------------------------------------------------ */

static void on_driver_event(const uvc_host_driver_event_data_t *event, void *ctx)
{
    if (event->type == UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED) {
        mgr_evt_t e = {
            .kind = EVT_CONNECTED,
            .addr = event->device_connected.dev_addr,
            .idx = event->device_connected.uvc_stream_index,
        };
        xQueueSend(s_evt_q, &e, 0);
    }
}

static void on_stream_event(const uvc_host_stream_event_data_t *event, void *ctx)
{
    switch (event->type) {
    case UVC_HOST_DEVICE_DISCONNECTED: {
        mgr_evt_t e = { .kind = EVT_DISCONNECTED };
        xQueueSend(s_evt_q, &e, 0);
        break;
    }
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGW(TAG, "USB transfer error %d", event->transfer_error.error);
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
        ESP_LOGW(TAG, "frame buffer overflow (frame bigger than frame_size)");
        break;
    default:
        break;
    }
}

static bool on_frame(const uvc_host_frame_t *frame, void *ctx)
{
    s_stats.rx_frames++;
    s_stats.rx_bytes += frame->data_len;
    s_last_rx = xTaskGetTickCount();
    rx_item_t item = { .frame = frame, .gen = s_gen };
    if (xQueueSendToBack(s_rx_q, &item, 0) != pdPASS) {
        s_stats.rx_dropped++;
        return true;        /* not queued: hand the buffer straight back to the driver */
    }
    return false;           /* decode task returns it with uvc_host_frame_return() */
}

static void on_out_release(const video_frame_t *frame, void *ctx)
{
    int idx = (int)(intptr_t)frame->priv;
    xQueueSend(s_out_free_q, &idx, 0);
}

/* ---- decoding ------------------------------------------------------------- */

#if APP_UVC_CTRL_TEST
/* Mean luma (0..255) over a 16x16-pixel grid; enough to see a control's effect. */
static uint8_t sample_luma(const uint8_t *buf, int w, int h, int stride)
{
    const uint16_t *px = (const uint16_t *)buf;
    uint32_t sum = 0, n = 0;
    for (int y = 8; y < h; y += 16) {
        for (int x = 8; x < w; x += 16) {
            uint16_t v = px[y * stride + x];
            sum += (((v >> 11) & 0x1f) << 3) + (((v >> 5) & 0x3f) << 3) + ((v & 0x1f) << 3);
            n++;
        }
    }
    return n ? (uint8_t)(sum / (n * 4)) : 0;
}
#endif

static void decode_and_publish(const uvc_host_frame_t *f)
{
    int idx;
    if (xQueueReceive(s_out_free_q, &idx, 0) != pdTRUE) {
        s_stats.no_buffer++;
        return;
    }
    jpeg_dec_result_t r;
    int64_t t0 = esp_timer_get_time();
    esp_err_t err = jpeg_dec_decode(f->data, f->data_len, s_out[idx], s_out_cap, &r);
    s_stats.decode_us += (uint32_t)(esp_timer_get_time() - t0);
    if (err != ESP_OK) {
        if ((s_stats.decode_errors++ % 30) == 0) {
            ESP_LOGW(TAG, "JPEG decode failed (%s), %u bytes", esp_err_to_name(err), (unsigned)f->data_len);
        }
        xQueueSend(s_out_free_q, &idx, 0);
        return;
    }
    s_stats.decoded++;
#if APP_UVC_CTRL_TEST
    s_stats.luma = sample_luma(s_out[idx], r.w, r.h, r.stride);
#endif
    video_frame_t out = {
        .data = s_out[idx],
        .w = r.w,
        .h = r.h,
        .stride = r.stride,
        .priv = (void *)(intptr_t)idx,
    };
    frame_mbox_publish(s_mbox, &out);
}

static void decode_task(void *arg)
{
    for (;;) {
        rx_item_t item;
        if (xQueueReceive(s_rx_q, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        xSemaphoreTake(s_lock, portMAX_DELAY);
        /* A frame of an already closed stream must not be touched (its memory is gone). */
        if (s_stream && item.gen == s_gen) {
            decode_and_publish(item.frame);
            uvc_host_frame_return(s_stream, (uvc_host_frame_t *)item.frame);
        }
        xSemaphoreGive(s_lock);
    }
}

/* ---- camera mode selection / stream management ----------------------------- */

/* Pick the frame interval (100 ns units) of `fi` closest to `want`. */
static uint32_t pick_interval(const uvc_host_frame_info_t *fi, uint32_t want)
{
    if (fi->interval_type == 0) {   /* continuous range */
        /* Malformed range: the UVC host walks min..max by step, so a zero step
         * would never terminate there. Request the camera's default FPS instead. */
        if (fi->interval_step == 0 || fi->interval_min > fi->interval_max) {
            return 0;
        }
        uint32_t clamped = want;
        if (clamped < fi->interval_min) {
            clamped = fi->interval_min;
        } else if (clamped > fi->interval_max) {
            clamped = fi->interval_max;
        }

        /* A continuous descriptor still permits only min + n * step. The UVC
         * host validates the requested FPS against those exact values. */
        uint32_t steps = (clamped - fi->interval_min) / fi->interval_step;
        uint32_t lower = fi->interval_min + steps * fi->interval_step;
        uint64_t upper64 = (uint64_t)lower + fi->interval_step;
        if (upper64 > fi->interval_max) {
            return lower;
        }
        uint32_t upper = (uint32_t)upper64;
        return clamped - lower <= upper - clamped ? lower : upper;
    }
    uint32_t best = fi->interval[0];
    for (int i = 1; i < fi->interval_type && i < CONFIG_UVC_INTERVAL_ARRAY_SIZE; i++) {
        uint32_t d = fi->interval[i] > want ? fi->interval[i] - want : want - fi->interval[i];
        uint32_t bd = best > want ? best - want : want - best;
        if (d < bd) {
            best = fi->interval[i];
        }
    }
    return best;
}

static void log_modes(const uvc_host_frame_info_t *list, size_t n)
{
    static const char *const names[] = { "default", "MJPEG", "YUY2", "H264", "H265" };
    for (size_t i = 0; i < n; i++) {
        const uvc_host_frame_info_t *fi = &list[i];
        char fps[64] = "";
        if (fi->interval_type == 0) {
            snprintf(fps, sizeof(fps), "%.1f-%.1f fps", 1e7 / fi->interval_max, 1e7 / fi->interval_min);
        } else {
            size_t used = 0;
            for (int k = 0; k < fi->interval_type && k < CONFIG_UVC_INTERVAL_ARRAY_SIZE && used < sizeof(fps) - 1; k++) {
                int written = snprintf(fps + used, sizeof(fps) - used, "%s%.0f",
                                       k ? "/" : "", 1e7 / fi->interval[k]);
                if (written < 0) {
                    break;
                }
                size_t available = sizeof(fps) - used;
                used += (size_t)written < available ? (size_t)written : available - 1;
            }
            if (used < sizeof(fps) - 1) {
                snprintf(fps + used, sizeof(fps) - used, " fps%s",
                         fi->interval_type > CONFIG_UVC_INTERVAL_ARRAY_SIZE ? " (+more)" : "");
            }
        }
        ESP_LOGI(TAG, "  mode %u: %-5s %ux%u  %s", (unsigned)i,
                 fi->format < sizeof(names) / sizeof(names[0]) ? names[fi->format] : "?", fi->h_res, fi->v_res, fps);
    }
}

/* Choose the MJPEG mode closest to the wanted size, then the nearest frame rate. */
static bool choose_mode(const uvc_host_frame_info_t *list, size_t n, uvc_host_stream_format_t *out)
{
    long best_score = -1;
    const uvc_host_frame_info_t *best = NULL;
    const long want_area = (long)APP_UVC_W * APP_UVC_H;

    for (size_t i = 0; i < n; i++) {
        const uvc_host_frame_info_t *fi = &list[i];
        if (fi->format != UVC_VS_FORMAT_MJPEG || fi->h_res > APP_UVC_MAX_W || fi->v_res > APP_UVC_MAX_H) {
            continue;
        }
        long area = (long)fi->h_res * fi->v_res;
        long score = area > want_area ? area - want_area : want_area - area;
        if (best_score < 0 || score < best_score) {
            best_score = score;
            best = fi;
        }
    }
    if (!best) {
        return false;
    }
    uint32_t interval = pick_interval(best, INTERVAL_100NS(APP_UVC_FPS));
    out->format = UVC_VS_FORMAT_MJPEG;
    out->h_res = best->h_res;
    out->v_res = best->v_res;
    out->fps = interval ? 1e7f / interval : 0;
    return true;
}

static void close_stream(void);

static esp_err_t try_open(uint8_t addr, uint8_t idx)
{
    size_t n = 0;
    if (uvc_host_get_frame_list(addr, idx, NULL, &n) != ESP_OK || n == 0) {
        return ESP_ERR_NOT_FOUND;      /* not a UVC device */
    }
    uvc_host_frame_info_t *list = calloc(n, sizeof(*list));
    ESP_RETURN_ON_FALSE(list, ESP_ERR_NO_MEM, TAG, "no memory");
    esp_err_t ret = uvc_host_get_frame_list(addr, idx, (uvc_host_frame_info_t (*)[])list, &n);
    if (ret != ESP_OK) {
        free(list);
        return ret;
    }

    ESP_LOGI(TAG, "UVC camera at USB address %u (stream %u): %u modes", addr, idx, (unsigned)n);
    log_modes(list, n);

    uvc_host_stream_format_t fmt;
    bool found = choose_mode(list, n, &fmt);
    free(list);
    if (!found) {
        ESP_LOGE(TAG, "camera has no usable MJPEG mode (<= %dx%d)", APP_UVC_MAX_W, APP_UVC_MAX_H);
        return ESP_ERR_NOT_SUPPORTED;
    }
    ESP_LOGI(TAG, "selected MJPEG %ux%u @ %.1f fps (wanted %dx%d @ %d)", fmt.h_res, fmt.v_res, fmt.fps,
             APP_UVC_W, APP_UVC_H, APP_UVC_FPS);

    const uvc_host_stream_config_t cfg = {
        .event_cb = on_stream_event,
        .frame_cb = on_frame,
        .user_ctx = NULL,
        .usb = {
            .dev_addr = addr,
            .vid = UVC_HOST_ANY_VID,
            .pid = UVC_HOST_ANY_PID,
            .uvc_stream_index = idx,
        },
        .vs_format = fmt,
        .advanced = {
            .number_of_frame_buffers = APP_UVC_FRAME_BUFS,
            .frame_size = 0,                        /* use the size negotiated with the camera */
            .frame_heap_caps = MALLOC_CAP_SPIRAM,
            .number_of_urbs = 3,
            .urb_size = 16 * 1024,
        },
    };
    uvc_host_stream_hdl_t hdl = NULL;
    ESP_RETURN_ON_ERROR(uvc_host_stream_open(&cfg, pdMS_TO_TICKS(2000), &hdl), TAG, "stream open failed");

    uvc_host_stream_format_t actual = { 0 };
    if (uvc_host_stream_format_get(hdl, &actual) == ESP_OK) {
        ESP_LOGI(TAG, "camera accepted %ux%u @ %.1f fps", actual.h_res, actual.v_res, actual.fps);
        s_stats.w = actual.h_res;
        s_stats.h = actual.v_res;
    }
#if APP_UVC_CTRL_PROBE
    /* Before streaming starts, so the control transfers cannot disturb it. */
    if (cam_uvc_ctrl_probe(addr, hdl) != ESP_OK) {
        ESP_LOGW(TAG, "image controls unavailable; streaming anyway");
    }
#endif

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_gen++;
    s_stream = hdl;
    xSemaphoreGive(s_lock);

    ret = uvc_host_stream_start(hdl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "stream start failed (%s)", esp_err_to_name(ret));
        /* The control probe above retains hdl too. Use the common close path
         * so both owners forget it before the driver frees the stream. */
        close_stream();
        return ret;
    }
    s_last_rx = xTaskGetTickCount();
    s_stats.streaming = true;
    ESP_LOGI(TAG, "streaming");
    return ESP_OK;
}

static void close_stream(void)
{
    /* Stop callbacks first. uvc_host_stream_stop() pauses the stream before
     * waiting for in-flight USB transfers, so no new user-owned frame can be
     * queued after it returns. Do not hold s_lock here: the decode task may
     * need it to return a frame that was already being processed. */
    xSemaphoreTake(s_lock, portMAX_DELAY);
    uvc_host_stream_hdl_t hdl = s_stream;
    xSemaphoreGive(s_lock);

    if (hdl) {
        esp_err_t err = uvc_host_stream_stop(hdl);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "stream stop failed (%s); continuing close", esp_err_to_name(err));
        }
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_stream = NULL;
    s_gen++;
    rx_item_t drop;
    unsigned returned = 0;
    while (xQueueReceive(s_rx_q, &drop, 0) == pdTRUE) {
        /* on_frame() returned false for every queued frame, so ownership is
         * ours until it is explicitly returned. The UVC driver refuses to
         * close a stream while even one such frame remains outstanding. */
        if (hdl && drop.frame) {
            esp_err_t err = uvc_host_frame_return(hdl, (uvc_host_frame_t *)drop.frame);
            if (err == ESP_OK) {
                returned++;
            } else {
                ESP_LOGE(TAG, "queued frame return failed (%s)", esp_err_to_name(err));
            }
        }
    }
    xSemaphoreGive(s_lock);

    if (returned) {
        ESP_LOGI(TAG, "returned %u queued frame(s) before close", returned);
    }

    s_stats.streaming = false;
    cam_uvc_ctrl_forget();
    if (hdl) {
        esp_err_t err = uvc_host_stream_close(hdl);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "stream close failed (%s)", esp_err_to_name(err));
        }
    }
}

/* Turn the USB-A 5 V off and on again. The camera drops off the bus and enumerates
 * afresh, exactly like unplugging it. This is the only reliable way back once its
 * control endpoint stops answering (uvc_host_stream_stop() also triggers that). */
static void power_cycle_camera(const char *why)
{
    ESP_LOGW(TAG, "%s: power-cycling the USB-A port", why);
    s_stats.power_cycles++;
    s_open_failures = 0;
    close_stream();
    bsp_feature_enable(BSP_FEATURE_USB, false);
    vTaskDelay(pdMS_TO_TICKS(APP_UVC_POWER_OFF_MS));
    bsp_feature_enable(BSP_FEATURE_USB, true);
}

/* A camera that is present but cannot be opened several times in a row is wedged. */
static void note_open_result(esp_err_t err)
{
    if (err == ESP_OK) {
        s_open_failures = 0;
    } else if (err != ESP_ERR_NOT_FOUND && ++s_open_failures >= APP_UVC_OPEN_RETRIES) {
        power_cycle_camera("camera keeps failing to open");
    }
}

/* Open the first UVC device found among the already enumerated USB devices. */
static void scan_and_open(void)
{
    uint8_t addrs[10];
    int count = 0;
    if (usb_host_device_addr_list_fill(sizeof(addrs), addrs, &count) != ESP_OK) {
        return;
    }
    for (int i = 0; i < count && !s_stream; i++) {
        note_open_result(try_open(addrs[i], 0));
    }
}

/* Streaming but no frame for APP_UVC_STALL_MS: the camera or the USB transfer
 * got stuck without a disconnect event. */
static bool stream_stalled(void)
{
    return s_stream && (TickType_t)(xTaskGetTickCount() - s_last_rx) > pdMS_TO_TICKS(APP_UVC_STALL_MS);
}

static void manager_task(void *arg)
{
    scan_and_open();        /* start-up: a camera may already be plugged in */
#if APP_UVC_STALL_TEST
    TickType_t stall_test_at = xTaskGetTickCount() + pdMS_TO_TICKS(20000);
    bool stall_test_done = false;
#endif

    bool cycle_pending = false;
    TickType_t cycle_at = 0;

    for (;;) {
        mgr_evt_t e;
        TickType_t wait = pdMS_TO_TICKS(s_stream ? APP_UVC_WATCHDOG_POLL_MS : APP_UVC_RESCAN_MS);
        if (cycle_pending) {
            int32_t left = (int32_t)(cycle_at - xTaskGetTickCount());
            TickType_t until = left > 0 ? (TickType_t)left : 0;
            wait = until < wait ? until : wait;
        }
        if (xQueueReceive(s_evt_q, &e, wait) == pdTRUE) {
            if (e.kind == EVT_DISCONNECTED) {
                ESP_LOGW(TAG, "camera disconnected");
                close_stream();
            } else if (e.kind == EVT_POWER_CYCLE) {
                /* Debounced: a burst of taps costs one restart. */
                cycle_pending = true;
                cycle_at = xTaskGetTickCount() + pdMS_TO_TICKS(APP_UVC_APPLY_DELAY_MS);
            } else if (!s_stream) {
                vTaskDelay(pdMS_TO_TICKS(300));     /* let the device settle after enumeration */
                note_open_result(try_open(e.addr, e.idx));
            }
        } else if (!s_stream) {
            scan_and_open();    /* safety net for missed connect events */
        } else if (stream_stalled()) {
            s_stats.stall_restarts++;
            power_cycle_camera("no frame for " STR(APP_UVC_STALL_MS) " ms");
        }
        if (cycle_pending && (int32_t)(xTaskGetTickCount() - cycle_at) >= 0) {
            cycle_pending = false;
            power_cycle_camera("applying camera settings");
        }
#if APP_UVC_STALL_TEST
        if (!stall_test_done && s_stream && (int32_t)(xTaskGetTickCount() - stall_test_at) >= 0) {
            stall_test_done = true;
            ESP_LOGW(TAG, "stall test: stopping the stream behind the manager's back");
            uvc_host_stream_stop(s_stream);
        }
#endif
    }
}

/* ---- public API -------------------------------------------------------------- */

void cam_uvc_request_power_cycle(void)
{
    mgr_evt_t e = { .kind = EVT_POWER_CYCLE };
    xQueueSend(s_evt_q, &e, 0);
}


esp_err_t cam_uvc_init(void)
{
    s_evt_q = xQueueCreate(8, sizeof(mgr_evt_t));
    s_rx_q = xQueueCreate(APP_UVC_FRAME_BUFS, sizeof(rx_item_t));
    s_out_free_q = xQueueCreate(APP_UVC_OUT_BUFS, sizeof(int));
    s_lock = xSemaphoreCreateMutex();
    s_mbox = frame_mbox_create(on_out_release, NULL);
    ESP_RETURN_ON_FALSE(s_evt_q && s_rx_q && s_out_free_q && s_lock && s_mbox, ESP_ERR_NO_MEM, TAG, "no memory");

    ESP_RETURN_ON_ERROR(cam_uvc_ctrl_init(), TAG, "control init failed");
    ESP_RETURN_ON_ERROR(jpeg_dec_init(), TAG, "JPEG decoder init failed");
    size_t want = jpeg_dec_output_size(APP_UVC_MAX_W, APP_UVC_MAX_H);
    for (int i = 0; i < APP_UVC_OUT_BUFS; i++) {
        s_out[i] = jpeg_dec_alloc_output(want, &s_out_cap);
        ESP_RETURN_ON_FALSE(s_out[i], ESP_ERR_NO_MEM, TAG, "no memory for output buffer");
        xQueueSend(s_out_free_q, &i, 0);
    }

    ESP_RETURN_ON_ERROR(bsp_usb_host_start(BSP_USB_HOST_POWER_MODE_USB_DEV, false), TAG, "USB host start failed");
    const uvc_host_driver_config_t drv = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = APP_TASK_UVC_DRIVER_PRIO,
        .xCoreID = 0,
        .create_background_task = true,
        .event_cb = on_driver_event,
        .user_ctx = NULL,
    };
    ESP_RETURN_ON_ERROR(uvc_host_install(&drv), TAG, "UVC driver install failed");
    return ESP_OK;
}

frame_mbox_t cam_uvc_mbox(void)
{
    return s_mbox;
}

esp_err_t cam_uvc_start(void)
{
    ESP_RETURN_ON_FALSE(xTaskCreate(decode_task, "uvc_decode", APP_TASK_UVC_DECODE_STACK, NULL,
                                    APP_TASK_UVC_DECODE_PRIO, NULL) == pdPASS &&
                        xTaskCreate(manager_task, "uvc_mgr", APP_TASK_UVC_MGR_STACK, NULL,
                                    APP_TASK_UVC_MGR_PRIO, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task create failed");
    return ESP_OK;
}

void cam_uvc_get_stats(cam_uvc_stats_t *out)
{
    *out = s_stats;
}
