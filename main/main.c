#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_config.h"
#include "cam_builtin.h"
#include "cam_uvc.h"
#include "display_comp.h"
#include "touch_input.h"
#include "camera_ui.h"
#include "cam_uvc_ctrl.h"

static const char *TAG = "main";

#if APP_CAM_CONTROL_TEST
static void camera_control_test_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(5000));
    cam_builtin_ctrl_info_t info;
    if (cam_builtin_ctrl_query(CAM_BUILTIN_CTRL_BRIGHTNESS, &info) != ESP_OK) {
        ESP_LOGE(TAG, "control test: brightness query failed");
        vTaskDelete(NULL);
    }
    int32_t original = info.current;
    int32_t test = original + 32 <= info.maximum ? original + 32 : original - 32;
    ESP_LOGI(TAG, "control test: brightness %" PRId32 " -> %" PRId32, original, test);
    if (cam_builtin_ctrl_set(CAM_BUILTIN_CTRL_BRIGHTNESS, test) != ESP_OK) {
        ESP_LOGE(TAG, "control test: set failed");
        vTaskDelete(NULL);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (cam_builtin_ctrl_query(CAM_BUILTIN_CTRL_BRIGHTNESS, &info) == ESP_OK) {
        ESP_LOGI(TAG, "control test: after 2s brightness=%" PRId32, info.current);
    }
    esp_err_t restore_err = cam_builtin_ctrl_set(CAM_BUILTIN_CTRL_BRIGHTNESS, original);
    vTaskDelay(pdMS_TO_TICKS(250));
    if (restore_err == ESP_OK && cam_builtin_ctrl_query(CAM_BUILTIN_CTRL_BRIGHTNESS, &info) == ESP_OK) {
        ESP_LOGI(TAG, "control test: restored brightness=%" PRId32, info.current);
    } else {
        ESP_LOGE(TAG, "control test: restore failed");
    }
    vTaskDelete(NULL);
}
#endif

#if APP_CAM_CCM_TEST
static void log_ccm(const char *label, const cam_builtin_ccm_t *ccm)
{
    ESP_LOGI(TAG, "%s CCM enable=%d [%.3f %.3f %.3f; %.3f %.3f %.3f; %.3f %.3f %.3f]",
             label, ccm->enable,
             ccm->matrix[0][0], ccm->matrix[0][1], ccm->matrix[0][2],
             ccm->matrix[1][0], ccm->matrix[1][1], ccm->matrix[1][2],
             ccm->matrix[2][0], ccm->matrix[2][1], ccm->matrix[2][2]);
}

static float clamp_ccm(float value)
{
    return value < -3.99f ? -3.99f : value > 3.99f ? 3.99f : value;
}

static void camera_ccm_test_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(5000));
    cam_builtin_ccm_t original;
    if (cam_builtin_ccm_get(&original) != ESP_OK) {
        ESP_LOGE(TAG, "CCM test: initial read failed; no change made");
        vTaskDelete(NULL);
    }
    log_ccm("CCM test: original", &original);

    cam_builtin_ccm_t test = original;
    test.enable = true;
    for (int row = 0; row < 3; row++) {
        test.matrix[row][0] = clamp_ccm(test.matrix[row][0] * 1.10f);
        test.matrix[row][2] = clamp_ccm(test.matrix[row][2] * 0.90f);
    }
    if (cam_builtin_ccm_set(&test) != ESP_OK) {
        ESP_LOGE(TAG, "CCM test: set failed");
        vTaskDelete(NULL);
    }
    cam_builtin_ccm_t current;
    if (cam_builtin_ccm_get(&current) == ESP_OK) {
        log_ccm("CCM test: immediate", &current);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (cam_builtin_ccm_get(&current) == ESP_OK) {
        log_ccm("CCM test: after 2s", &current);
    }
    esp_err_t restore_err = cam_builtin_ccm_set(&original);
    if (restore_err == ESP_OK && cam_builtin_ccm_get(&current) == ESP_OK) {
        log_ccm("CCM test: restored", &current);
    } else {
        ESP_LOGE(TAG, "CCM test: restore failed");
    }
    vTaskDelete(NULL);
}
#endif

#if APP_UVC_CTRL_TEST
/* Wait, then report read-back value, MJPEG bytes/frame and mean luma. */
static void uvc_test_measure(const char *label, cam_uvc_ctrl_t ctrl)
{
    vTaskDelay(pdMS_TO_TICKS(1500));    /* let the camera settle */
    cam_uvc_stats_t a, b;
    cam_uvc_get_stats(&a);
    vTaskDelay(pdMS_TO_TICKS(1000));
    cam_uvc_get_stats(&b);
    int32_t cur = 0;
    esp_err_t err = cam_uvc_ctrl_read(ctrl, &cur);
    uint32_t frames = b.rx_frames - a.rx_frames;
    ESP_LOGI(TAG, "uvc test %-22s read %6" PRId32 "%s | %5u B/frame | luma %3u",
             label, cur, err == ESP_OK ? "" : " (read failed)",
             frames ? (unsigned)((b.rx_bytes - a.rx_bytes) / frames) : 0, b.luma);
}

static void uvc_test_sweep(const char *name, cam_uvc_ctrl_t ctrl, int32_t low, int32_t high)
{
    cam_uvc_ctrl_info_t info;
    if (!cam_uvc_ctrl_get_info(ctrl, &info)) {
        ESP_LOGW(TAG, "uvc test %s: unavailable", name);
        return;
    }
    char label[32];
    int32_t original = info.current;
    snprintf(label, sizeof(label), "%s original", name);
    uvc_test_measure(label, ctrl);
    const int32_t values[2] = { high, low };
    for (int i = 0; i < 2; i++) {
        esp_err_t err = cam_uvc_ctrl_set(ctrl, values[i]);
        snprintf(label, sizeof(label), "%s=%" PRId32 "%s", name, values[i], err == ESP_OK ? "" : " FAIL");
        uvc_test_measure(label, ctrl);
    }
    esp_err_t err = cam_uvc_ctrl_set(ctrl, original);
    ESP_LOGI(TAG, "uvc test %s restored to %" PRId32 " (%s)", name, original, esp_err_to_name(err));
}

static void uvc_ctrl_test_task(void *arg)
{
    cam_uvc_stats_t st;
    do {
        vTaskDelay(pdMS_TO_TICKS(1000));
        cam_uvc_get_stats(&st);
    } while (!st.streaming);
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "uvc test: start (watch the right pane)");

    static const struct { const char *name; cam_uvc_ctrl_t ctrl; } sweeps[] = {
        { "brightness", CAM_UVC_CTRL_BRIGHTNESS }, { "contrast", CAM_UVC_CTRL_CONTRAST },
        { "saturation", CAM_UVC_CTRL_SATURATION }, { "gamma", CAM_UVC_CTRL_GAMMA },
        { "sharpness", CAM_UVC_CTRL_SHARPNESS },
    };
    for (size_t i = 0; i < sizeof(sweeps) / sizeof(sweeps[0]); i++) {
        cam_uvc_ctrl_info_t info;
        if (cam_uvc_ctrl_get_info(sweeps[i].ctrl, &info)) {
            uvc_test_sweep(sweeps[i].name, sweeps[i].ctrl, info.minimum, info.maximum);
        }
    }

    /* White balance: manual temperature needs auto off; restore auto afterwards. */
    cam_uvc_ctrl_info_t wb_auto;
    if (cam_uvc_ctrl_get_info(CAM_UVC_CTRL_WB_TEMP_AUTO, &wb_auto)) {
        esp_err_t err = cam_uvc_ctrl_set(CAM_UVC_CTRL_WB_TEMP_AUTO, 0);
        ESP_LOGI(TAG, "uvc test wb auto off (%s)", esp_err_to_name(err));
        cam_uvc_ctrl_info_t wb;
        if (err == ESP_OK && cam_uvc_ctrl_get_info(CAM_UVC_CTRL_WB_TEMP, &wb)) {
            uvc_test_sweep("wb_temp", CAM_UVC_CTRL_WB_TEMP, wb.minimum, wb.maximum);
        }
        err = cam_uvc_ctrl_set(CAM_UVC_CTRL_WB_TEMP_AUTO, wb_auto.current);
        ESP_LOGI(TAG, "uvc test wb auto restored to %" PRId32 " (%s)", wb_auto.current, esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "uvc test: done");
    vTaskDelete(NULL);
}
#endif

/* "No signal" colours (RGB565) */
#define COLOR_NO_CAM_BUILTIN    0x4000  /* dark red */
#define COLOR_NO_CAM_UVC        0x0010  /* dark blue */

static void log_stats(void)
{
    static frame_mbox_stats_t prev_builtin, prev_uvc;
    static display_comp_stats_t prev_disp;
    static cam_uvc_stats_t prev_rx;
    frame_mbox_stats_t builtin, uvc;
    display_comp_stats_t disp;
    cam_uvc_stats_t rx;

    frame_mbox_get_stats(cam_builtin_mbox(), &builtin);
    frame_mbox_get_stats(cam_uvc_mbox(), &uvc);
    display_comp_get_stats(&disp);
    cam_uvc_get_stats(&rx);

    float secs = APP_TASK_STATS_PERIOD_MS / 1000.0f;
    ESP_LOGI(TAG, "builtin %.1f fps | uvc %s %ux%u rx %.1f fps dec %.1f fps (err %u, nobuf %u, drop %u) | "
                  "display %.1f fps | PSRAM %uK int %uK",
             (builtin.published - prev_builtin.published) / secs,
             rx.streaming ? "ON" : "off", rx.w, rx.h,
             (rx.rx_frames - prev_rx.rx_frames) / secs, (uvc.published - prev_uvc.published) / secs,
             (unsigned)(rx.decode_errors - prev_rx.decode_errors), (unsigned)(rx.no_buffer - prev_rx.no_buffer),
             (unsigned)(rx.rx_dropped - prev_rx.rx_dropped),
             (disp.flips - prev_disp.flips) / secs,
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
    /* Profiling: average cost per operation over the last period */
    uint32_t srm_n = disp.srm_calls - prev_disp.srm_calls;
    uint32_t fill_n = disp.fill_calls - prev_disp.fill_calls;
    uint32_t flips = disp.flips - prev_disp.flips;
    uint32_t dec_n = rx.decoded - prev_rx.decoded;
    ESP_LOGI(TAG, "prof: srm %u us x%u | fill %u us x%u | flip call %u us, wait %u us (per flip) | "
                  "jpeg %u us, %u B/frame",
             srm_n ? (unsigned)((disp.srm_us - prev_disp.srm_us) / srm_n) : 0, (unsigned)srm_n,
             fill_n ? (unsigned)((disp.fill_us - prev_disp.fill_us) / fill_n) : 0, (unsigned)fill_n,
             flips ? (unsigned)((disp.flip_call_us - prev_disp.flip_call_us) / flips) : 0,
             flips ? (unsigned)((disp.flip_wait_us - prev_disp.flip_wait_us) / flips) : 0,
             dec_n ? (unsigned)((rx.decode_us - prev_rx.decode_us) / dec_n) : 0,
             (rx.rx_frames - prev_rx.rx_frames)
                 ? (unsigned)((rx.rx_bytes - prev_rx.rx_bytes) / (rx.rx_frames - prev_rx.rx_frames)) : 0);
    prev_builtin = builtin;
    prev_uvc = uvc;
    prev_disp = disp;
    prev_rx = rx;
}

#if APP_DISPLAY_BENCH
/* PSRAM bandwidth before anything else (display, camera) is running */
static void psram_baseline(void)
{
    const size_t n = 1024 * 1024;
    uint8_t *a = heap_caps_aligned_calloc(64, 1, n, MALLOC_CAP_SPIRAM);
    uint8_t *b = heap_caps_aligned_calloc(64, 1, n, MALLOC_CAP_SPIRAM);
    if (a && b) {
        int64_t t0 = esp_timer_get_time();
        for (int i = 0; i < 8; i++) {
            memcpy(b, a, n);
        }
        int64_t t1 = esp_timer_get_time();
        for (int i = 0; i < 8; i++) {
            memset(b, i, n);
        }
        int64_t t2 = esp_timer_get_time();
        ESP_LOGI(TAG, "baseline (no display/camera): memcpy psram->psram %d MB/s, memset psram %d MB/s",
                 (int)(8.0 * n / (t1 - t0)), (int)(8.0 * n / (t2 - t1)));
    }
    free(a);
    free(b);
}
#endif

void app_main(void)
{
#if APP_DISPLAY_BENCH
    psram_baseline();
#endif
    ESP_ERROR_CHECK(display_comp_init());
    esp_err_t touch_err = touch_input_init();
    if (touch_err != ESP_OK) {
        ESP_LOGW(TAG, "touch disabled (%s)", esp_err_to_name(touch_err));
    }
    ESP_ERROR_CHECK(cam_builtin_init());
    esp_err_t ui_err = camera_ui_init();
    if (ui_err != ESP_OK) {
        ESP_LOGW(TAG, "camera UI disabled (%s)", esp_err_to_name(ui_err));
    } else {
        touch_input_set_press_callback(camera_ui_on_touch);
    }
    ESP_ERROR_CHECK(cam_uvc_init());

    ESP_ERROR_CHECK(display_comp_attach(DISP_PANE_LEFT, cam_builtin_mbox(), APP_CAM_BUILTIN_ROT_DEG, COLOR_NO_CAM_BUILTIN));
    ESP_ERROR_CHECK(display_comp_attach(DISP_PANE_RIGHT, cam_uvc_mbox(), APP_UVC_ROT_DEG, COLOR_NO_CAM_UVC));

    ESP_ERROR_CHECK(display_comp_start());
    ESP_ERROR_CHECK(cam_builtin_start());
    ESP_ERROR_CHECK(cam_uvc_start());
    if (touch_err == ESP_OK) {
        touch_err = touch_input_start();
        if (touch_err != ESP_OK) {
            ESP_LOGW(TAG, "touch task disabled (%s)", esp_err_to_name(touch_err));
        }
    }
#if APP_CAM_CONTROL_TEST
    if (xTaskCreate(camera_control_test_task, "cam_ctrl_test", 3072, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "control test task create failed");
    }
#endif
#if APP_CAM_CCM_TEST
    if (xTaskCreate(camera_ccm_test_task, "cam_ccm_test", 3072, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "CCM test task create failed");
    }
#endif
#if APP_UVC_CTRL_TEST
    if (xTaskCreate(uvc_ctrl_test_task, "uvc_ctrl_test", 4096, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "UVC control test task create failed");
    }
#endif
    ESP_LOGI(TAG, "running");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(APP_TASK_STATS_PERIOD_MS));
        log_stats();
    }
}
