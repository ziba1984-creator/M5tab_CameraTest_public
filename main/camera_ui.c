#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "app_config.h"
#include "cam_builtin.h"
#include "cam_uvc_ctrl.h"
#include "camera_ui.h"
#include "display_comp.h"
#include "recorder.h"

static const char *TAG = "camera_ui";

#define UI_SRC_W       640
#define UI_SRC_H        40
#define UI_BUTTONS      11      /* target, 4 x (-, +), reset, record */
#define UI_GROUPS        4      /* B, C, S, W: each a "-" / "+" pair */
#define UI_BTN_TARGET    0
#define UI_BTN_RESET     9
#define UI_BTN_REC      10
#define UI_REC_POLL_MS 500      /* recording state and elapsed time refresh */
#define UI_FONT_SCALE    2
#define UI_LABEL_Y       4
#define UI_VALUE_Y      22
#define UI_CENTER_Y     13
#define UI_NONE         (-1)
#define UI_EVT_UVC      100     /* queue item: UVC controls appeared or vanished */

#define COLOR_BAR       0x18e3
#define COLOR_GROUP_A   0x2104
#define COLOR_GROUP_B   0x2945
#define COLOR_RESET     0x6000
#define COLOR_PRESSED   0x5acb
#define COLOR_RESET_ON  0xb000
#define COLOR_TARGET_IN 0x0012  /* dark blue: built-in camera */
#define COLOR_TARGET_UV 0x0300  /* dark green: UVC camera */
#define COLOR_LABEL     0xffff
#define COLOR_VALUE     0xffe0  /* yellow */
#define COLOR_DIVIDER   0x18e3
#define COLOR_REC_IDLE  0x3000  /* dark red */
#define COLOR_REC_ON    0xd000  /* bright red: recording */
#define COLOR_REC_FAIL  0x8200  /* brown: no card or write error */

typedef enum { TARGET_BUILTIN, TARGET_UVC } target_t;

/* Built-in: B, C, S are V4L2 controls; W (warmth) is the CCM step, handled separately. */
static const cam_builtin_ctrl_t s_group_ctrl[UI_GROUPS - 1] = {
    CAM_BUILTIN_CTRL_BRIGHTNESS, CAM_BUILTIN_CTRL_CONTRAST, CAM_BUILTIN_CTRL_SATURATION
};

/* UVC: W is the white balance temperature (manual only, auto is switched off first). */
static const cam_uvc_ctrl_t s_uvc_ctrl[UI_GROUPS] = {
    CAM_UVC_CTRL_BRIGHTNESS, CAM_UVC_CTRL_CONTRAST, CAM_UVC_CTRL_SATURATION, CAM_UVC_CTRL_WB_TEMP
};
static const int32_t s_uvc_step[UI_GROUPS] = { 8, 6, 16, 500 };

/* Two bar images: the display reads s_buf[s_front] under s_lock while the UI
 * task renders the other one, so the PPA never sees a half-drawn bar. */
static uint16_t *s_buf[2];
static int s_front;
static video_frame_t s_frame;
static SemaphoreHandle_t s_lock;
static QueueHandle_t s_queue;

static target_t s_target = TARGET_BUILTIN;
static cam_builtin_ctrl_info_t s_info[UI_GROUPS - 1];
static int32_t s_value[UI_GROUPS - 1];
static cam_builtin_ccm_t s_wb_base;
static int s_warmth;

/* UVC values as last known (cache of cam_uvc_ctrl; refreshed on events and taps). */
static bool s_uvc_ok[UI_GROUPS];
static cam_uvc_ctrl_info_t s_uvc[UI_GROUPS];
static bool s_uvc_auto_ok;
static cam_uvc_ctrl_info_t s_uvc_auto;

static int s_pressed = UI_NONE;
static uint32_t s_render_us_max;
static recorder_stats_t s_rec;      /* as last drawn */

static const char *const s_labels[UI_BUTTONS] = {
    "", "B-", "B+", "C-", "C+", "S-", "S+", "W-", "W+", "RST", "REC"
};

/* Five-bit rows, top to bottom. Only glyphs used by the compact control bar. */
static void glyph_rows(char ch, uint8_t row[7])
{
    static const uint8_t blank[7] = {0};
    static const uint8_t digits[10][7] = {
        {14,17,19,21,25,17,14}, {4,12,4,4,4,4,14}, {14,17,1,2,4,8,31}, {31,2,4,2,1,17,14},
        {2,6,10,18,31,2,2}, {31,16,30,1,1,17,14}, {6,8,16,30,17,17,14}, {31,1,2,4,8,8,8},
        {14,17,17,14,17,17,14}, {14,17,17,15,1,2,12},
    };
    const uint8_t *src = blank;
    static const uint8_t a[7] = {14,17,17,31,17,17,17};
    static const uint8_t b[7] = {30,17,17,30,17,17,30};
    static const uint8_t c[7] = {14,17,16,16,16,17,14};
    static const uint8_t d[7] = {28,18,17,17,17,18,28};
    static const uint8_t e[7] = {31,16,16,30,16,16,31};
    static const uint8_t i[7] = {14,4,4,4,4,4,14};
    static const uint8_t n[7] = {17,25,21,19,17,17,17};
    static const uint8_t o[7] = {14,17,17,17,17,17,14};
    static const uint8_t r[7] = {30,17,17,30,20,18,17};
    static const uint8_t s[7] = {15,16,16,14,1,1,30};
    static const uint8_t t[7] = {31,4,4,4,4,4,4};
    static const uint8_t u[7] = {17,17,17,17,17,17,14};
    static const uint8_t v[7] = {17,17,17,17,17,10,4};
    static const uint8_t w[7] = {17,17,17,21,21,21,10};
    static const uint8_t plus[7] = {0,4,4,31,4,4,0};
    static const uint8_t minus[7] = {0,0,0,31,0,0,0};
    static const uint8_t colon[7] = {0,4,4,0,4,4,0};
    switch (ch) {
    case 'A': src = a; break; case 'B': src = b; break; case 'C': src = c; break;
    case 'D': src = d; break; case 'E': src = e; break;
    case 'I': src = i; break; case 'N': src = n; break; case 'O': src = o; break;
    case 'R': src = r; break; case 'S': src = s; break; case 'T': src = t; break;
    case 'U': src = u; break; case 'V': src = v; break; case 'W': src = w; break;
    case '+': src = plus; break; case '-': src = minus; break; case ':': src = colon; break;
    default:
        if (ch >= '0' && ch <= '9') src = digits[ch - '0'];
        break;
    }
    memcpy(row, src, 7);
}

static void put_rect(uint16_t *px, int x, int y, int w, int h, uint16_t color)
{
    if (x < 0 || y < 0 || x + w > UI_SRC_W || y + h > UI_SRC_H) return;
    for (int yy = y; yy < y + h; yy++) {
        for (int xx = x; xx < x + w; xx++) px[yy * UI_SRC_W + xx] = color;
    }
}

static void put_text(uint16_t *px, int x, int y, const char *text, uint16_t color)
{
    for (; *text; text++, x += 6 * UI_FONT_SCALE) {
        uint8_t rows[7];
        glyph_rows(*text, rows);
        for (int gy = 0; gy < 7; gy++) {
            for (int gx = 0; gx < 5; gx++) {
                if (rows[gy] & (1 << (4 - gx))) {
                    put_rect(px, x + gx * UI_FONT_SCALE, y + gy * UI_FONT_SCALE,
                             UI_FONT_SCALE, UI_FONT_SCALE, color);
                }
            }
        }
    }
}

/* Draw text horizontally centred in [x0, x1). */
static void put_text_centered(uint16_t *px, int x0, int x1, int y, const char *text, uint16_t color)
{
    int tw = ((int)strlen(text) * 6 - 1) * UI_FONT_SCALE;
    put_text(px, x0 + (x1 - x0 - tw) / 2, y, text, color);
}

static int button_x(int i)
{
    return i * UI_SRC_W / UI_BUTTONS;
}

/* Current value of group g for the selected target, as shown in the bar. */
static void format_value(int g, char *text, size_t size)
{
    if (s_target == TARGET_BUILTIN) {
        if (g == UI_GROUPS - 1) {
            snprintf(text, size, s_warmth > 0 ? "+%d" : "%d", s_warmth);
        } else if (s_group_ctrl[g] == CAM_BUILTIN_CTRL_BRIGHTNESS) {
            snprintf(text, size, s_value[g] > 0 ? "+%" PRId32 : "%" PRId32, s_value[g]);
        } else {
            snprintf(text, size, "%" PRId32, s_value[g]);
        }
    } else if (!s_uvc_ok[g]) {
        snprintf(text, size, "--");
    } else if (g == UI_GROUPS - 1 && s_uvc_auto_ok && s_uvc_auto.current) {
        snprintf(text, size, "AUTO");
    } else if (s_uvc_ctrl[g] == CAM_UVC_CTRL_BRIGHTNESS) {
        snprintf(text, size, s_uvc[g].current > 0 ? "+%" PRId32 : "%" PRId32, s_uvc[g].current);
    } else {
        snprintf(text, size, "%" PRId32, s_uvc[g].current);
    }
}

/* REC: red while recording, with the elapsed time underneath; NOSD / ERR when
 * the last start found no card or the last recording hit a write error. */
static void render_rec(uint16_t *px)
{
    int x0 = button_x(UI_BTN_REC);
    uint16_t bg = s_pressed == UI_BTN_REC ? COLOR_PRESSED
                : s_rec.state == RECORDER_RECORDING ? COLOR_REC_ON
                : s_rec.state == RECORDER_IDLE ? COLOR_REC_IDLE : COLOR_REC_FAIL;
    put_rect(px, x0 + 1, 1, UI_SRC_W - x0 - 2, UI_SRC_H - 2, bg);
    put_text_centered(px, x0, UI_SRC_W, UI_LABEL_Y, s_labels[UI_BTN_REC], COLOR_LABEL);

    char text[8] = "";
    if (s_rec.state == RECORDER_RECORDING) {
        snprintf(text, sizeof(text), "%" PRIu32 ":%02" PRIu32, s_rec.seconds / 60 % 100, s_rec.seconds % 60);
    } else if (s_rec.state == RECORDER_NO_CARD) {
        snprintf(text, sizeof(text), "NOSD");
    } else if (s_rec.state == RECORDER_ERROR) {
        snprintf(text, sizeof(text), "ERR");
    }
    put_text_centered(px, x0, UI_SRC_W, UI_VALUE_Y, text, COLOR_VALUE);
}

/* Target switch, then each group drawn as one stepper ("-" and "+" labels on
 * top, the current value centred across both buttons underneath), then reset
 * and record. */
static void render_bar(uint16_t *px)
{
    put_rect(px, 0, 0, UI_SRC_W, UI_SRC_H, COLOR_BAR);

    int tx1 = button_x(UI_BTN_TARGET + 1);
    uint16_t tbg = s_pressed == UI_BTN_TARGET ? COLOR_PRESSED
                 : s_target == TARGET_UVC ? COLOR_TARGET_UV : COLOR_TARGET_IN;
    put_rect(px, 1, 1, tx1 - 2, UI_SRC_H - 2, tbg);
    put_text_centered(px, 0, tx1, UI_CENTER_Y, s_target == TARGET_UVC ? "UVC" : "IN", COLOR_LABEL);

    for (int g = 0; g < UI_GROUPS; g++) {
        int bm = 1 + 2 * g, bp = bm + 1;
        int x0 = button_x(bm), xm = button_x(bp), x1 = button_x(bp + 1);
        uint16_t bg = (g & 1) ? COLOR_GROUP_B : COLOR_GROUP_A;
        put_rect(px, x0 + 1, 1, x1 - x0 - 2, UI_SRC_H - 2, bg);
        if (s_pressed == bm) put_rect(px, x0 + 1, 1, xm - x0 - 1, UI_SRC_H - 2, COLOR_PRESSED);
        if (s_pressed == bp) put_rect(px, xm, 1, x1 - xm - 1, UI_SRC_H - 2, COLOR_PRESSED);
        put_rect(px, xm, 3, 1, UI_VALUE_Y - 5, COLOR_DIVIDER);
        put_text_centered(px, x0, xm, UI_LABEL_Y, s_labels[bm], COLOR_LABEL);
        put_text_centered(px, xm, x1, UI_LABEL_Y, s_labels[bp], COLOR_LABEL);

        char text[8];
        format_value(g, text, sizeof(text));
        put_text_centered(px, x0, x1, UI_VALUE_Y, text, COLOR_VALUE);
    }

    int x0 = button_x(UI_BTN_RESET), x1 = button_x(UI_BTN_RESET + 1);
    put_rect(px, x0 + 1, 1, x1 - x0 - 2, UI_SRC_H - 2,
             s_pressed == UI_BTN_RESET ? COLOR_RESET_ON : COLOR_RESET);
    put_text_centered(px, x0, x1, UI_CENTER_Y, s_labels[UI_BTN_RESET], COLOR_LABEL);

    render_rec(px);
}

/* Render into the back image, swap it in and ask the display for a redraw. */
static void publish_bar(void)
{
    int back = s_front ^ 1;
    int64_t t0 = esp_timer_get_time();
    render_bar(s_buf[back]);
    uint32_t us = (uint32_t)(esp_timer_get_time() - t0);
    if (us > s_render_us_max) {
        s_render_us_max = us;
        ESP_LOGI(TAG, "bar render %" PRIu32 " us (new max)", us);
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_front = back;
    s_frame.data = (uint8_t *)s_buf[back];
    xSemaphoreGive(s_lock);
    display_comp_request_redraw();
}

/* ---- built-in camera -------------------------------------------------------- */

static void adjust(int group, int direction)
{
    cam_builtin_ctrl_t ctrl = s_group_ctrl[group];
    const cam_builtin_ctrl_info_t *info = &s_info[group];
    int32_t quantum = ctrl == CAM_BUILTIN_CTRL_BRIGHTNESS ? 8 : 16;
    int32_t value = s_value[group] + direction * quantum;
    if (value < info->minimum) value = info->minimum;
    if (value > info->maximum) value = info->maximum;
    if (cam_builtin_ctrl_set(ctrl, value) == ESP_OK) {
        s_value[group] = value;
        ESP_LOGI(TAG, "control %d -> %" PRId32, ctrl, value);
    }
}

static void set_warmth(int value)
{
    if (value < -5) value = -5;
    if (value > 5) value = 5;
    cam_builtin_ccm_t ccm = s_wb_base;
    float red = 1.0f;
    float blue = 1.0f;
    if (value != 0) {
        ccm.enable = true;
        red += value * 0.05f;
        blue -= value * 0.05f;
        for (int row = 0; row < 3; row++) {
            ccm.matrix[row][0] *= red;
            ccm.matrix[row][2] *= blue;
        }
    }
    if (cam_builtin_ccm_set(&ccm) == ESP_OK) {
        s_warmth = value;
        ESP_LOGI(TAG, "warmth -> %d (red %.2f, blue %.2f)", value, red, blue);
    }
}

static void reset_all(void)
{
    for (int g = 0; g < UI_GROUPS - 1; g++) {
        if (cam_builtin_ctrl_set(s_group_ctrl[g], s_info[g].default_value) == ESP_OK) {
            s_value[g] = s_info[g].default_value;
        }
    }
    set_warmth(0);
    ESP_LOGI(TAG, "controls reset: B=%" PRId32 " C=%" PRId32 " S=%" PRId32 " W=%d",
             s_value[0], s_value[1], s_value[2], s_warmth);
}

/* ---- UVC camera ----------------------------------------------------------------- */

static void uvc_refresh(void)
{
    for (int g = 0; g < UI_GROUPS; g++) {
        s_uvc_ok[g] = cam_uvc_ctrl_get_info(s_uvc_ctrl[g], &s_uvc[g]);
    }
    s_uvc_auto_ok = cam_uvc_ctrl_get_info(CAM_UVC_CTRL_WB_TEMP_AUTO, &s_uvc_auto);
}

static void uvc_on_change(void)
{
    int evt = UI_EVT_UVC;
    if (xQueueSend(s_queue, &evt, 0) != pdTRUE) {
        ESP_LOGW(TAG, "busy, UVC refresh dropped");
    }
}

/* ESP_ERR_NOT_FINISHED: accepted, applied when the camera restarts (cam_uvc_ctrl_set). */
static bool uvc_accepted(esp_err_t err)
{
    return err == ESP_OK || err == ESP_ERR_NOT_FINISHED;
}

static void uvc_adjust(int group, int direction)
{
    if (!s_uvc_ok[group]) {
        ESP_LOGW(TAG, "uvc control %d unavailable", s_uvc_ctrl[group]);
        return;
    }
    cam_uvc_ctrl_t ctrl = s_uvc_ctrl[group];
    if (ctrl == CAM_UVC_CTRL_WB_TEMP && s_uvc_auto_ok && s_uvc_auto.current) {
        /* Manual temperature needs auto white balance off. Continue from the
         * temperature the camera had chosen automatically. */
        if (!uvc_accepted(cam_uvc_ctrl_set(CAM_UVC_CTRL_WB_TEMP_AUTO, 0))) {
            uvc_refresh();
            return;
        }
        int32_t now;
        if (cam_uvc_ctrl_read(ctrl, &now) == ESP_OK) {
            ESP_LOGI(TAG, "uvc auto white balance off at %" PRId32 " K", now);
        }
        uvc_refresh();
    }
    const cam_uvc_ctrl_info_t *info = &s_uvc[group];
    int32_t value = info->current + direction * s_uvc_step[group];
    if (value < info->minimum) value = info->minimum;
    if (value > info->maximum) value = info->maximum;
    esp_err_t err = cam_uvc_ctrl_set(ctrl, value);
    uvc_refresh();
    if (uvc_accepted(err)) {
        ESP_LOGI(TAG, "uvc control %d -> %" PRId32 "%s", ctrl, value,
                 err == ESP_OK ? "" : " (on camera restart)");
    }
}

static void uvc_reset(void)
{
    for (int g = 0; g < UI_GROUPS - 1; g++) {
        if (s_uvc_ok[g]) {
            cam_uvc_ctrl_set(s_uvc_ctrl[g], s_uvc[g].default_value);
        }
    }
    if (s_uvc_auto_ok) {
        cam_uvc_ctrl_set(CAM_UVC_CTRL_WB_TEMP_AUTO, s_uvc_auto.default_value);
    }
    uvc_refresh();
    ESP_LOGI(TAG, "uvc controls reset: B=%" PRId32 " C=%" PRId32 " S=%" PRId32 " WB auto=%" PRId32,
             s_uvc[0].current, s_uvc[1].current, s_uvc[2].current, s_uvc_auto.current);
}

/* ---- task --------------------------------------------------------------------------- */

static void apply_button(int button)
{
    if (button == UI_BTN_REC) {
        recorder_stats_t st;
        recorder_get_stats(&st);
        if (st.state == RECORDER_RECORDING) {
            recorder_stop();
        } else {
            esp_err_t err = recorder_start();
            ESP_LOGI(TAG, "record start: %s", esp_err_to_name(err));
        }
        recorder_get_stats(&s_rec);
        return;
    }
    if (button == UI_BTN_TARGET) {
        s_target = s_target == TARGET_BUILTIN ? TARGET_UVC : TARGET_BUILTIN;
        if (s_target == TARGET_UVC) {
            uvc_refresh();
        }
        ESP_LOGI(TAG, "target -> %s", s_target == TARGET_UVC ? "UVC" : "built-in");
        return;
    }
    if (button == UI_BTN_RESET) {
        if (s_target == TARGET_UVC) {
            uvc_reset();
        } else {
            reset_all();
        }
        return;
    }
    int group = (button - 1) / 2;
    int direction = ((button - 1) & 1) ? 1 : -1;
    if (s_target == TARGET_UVC) {
        uvc_adjust(group, direction);
    } else if (group == UI_GROUPS - 1) {
        set_warmth(s_warmth + direction);
    } else {
        adjust(group, direction);
    }
}

/* Owns every camera-control ioctl / USB control transfer and every redraw of
 * the bar, so the touch task only has to post the button index. */
static void ui_task(void *arg)
{
    TickType_t highlight_end = 0;
    for (;;) {
        /* Wake at least every UI_REC_POLL_MS to follow the recorder (elapsed
         * time, or a recording the writer stopped on an error). */
        TickType_t wait = pdMS_TO_TICKS(UI_REC_POLL_MS);
        if (s_pressed != UI_NONE) {
            int32_t left = (int32_t)(highlight_end - xTaskGetTickCount());
            wait = left <= 0 ? 0 : (TickType_t)left < wait ? (TickType_t)left : wait;
        }
        int button;
        if (xQueueReceive(s_queue, &button, wait) == pdTRUE) {
            if (button == UI_EVT_UVC) {
                uvc_refresh();
                publish_bar();
                continue;
            }
            apply_button(button);
            s_pressed = button;
            highlight_end = xTaskGetTickCount() + pdMS_TO_TICKS(APP_UI_HIGHLIGHT_MS);
            publish_bar();
            continue;
        }
        bool redraw = false;
        if (s_pressed != UI_NONE && (int32_t)(highlight_end - xTaskGetTickCount()) <= 0) {
            s_pressed = UI_NONE;
            redraw = true;
        }
        recorder_stats_t st;
        recorder_get_stats(&st);
        if (st.state != s_rec.state || st.seconds != s_rec.seconds) {
            s_rec = st;
            redraw = true;
        }
        if (redraw) {
            publish_bar();
        }
    }
}

esp_err_t camera_ui_init(void)
{
    for (int g = 0; g < UI_GROUPS - 1; g++) {
        ESP_RETURN_ON_ERROR(cam_builtin_ctrl_query(s_group_ctrl[g], &s_info[g]), TAG,
                            "control %d unavailable", s_group_ctrl[g]);
        s_value[g] = s_info[g].current;
    }
    ESP_RETURN_ON_ERROR(cam_builtin_ccm_get(&s_wb_base), TAG, "CCM baseline unavailable");
    for (int i = 0; i < 2; i++) {
        s_buf[i] = heap_caps_aligned_calloc(64, UI_SRC_W * UI_SRC_H, sizeof(uint16_t),
                                            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        ESP_RETURN_ON_FALSE(s_buf[i], ESP_ERR_NO_MEM, TAG, "no overlay memory");
    }
    s_lock = xSemaphoreCreateMutex();
    s_queue = xQueueCreate(4, sizeof(int));
    ESP_RETURN_ON_FALSE(s_lock && s_queue, ESP_ERR_NO_MEM, TAG, "no memory");

    recorder_get_stats(&s_rec);
    render_bar(s_buf[0]);
    s_front = 0;
    s_frame = (video_frame_t){ .data = (uint8_t *)s_buf[0], .w = UI_SRC_W, .h = UI_SRC_H, .stride = UI_SRC_W };
    ESP_RETURN_ON_FALSE(xTaskCreate(ui_task, "camera_ui", APP_TASK_UI_STACK, NULL,
                                    APP_TASK_UI_PRIO, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task create failed");
    cam_uvc_ctrl_set_notify(uvc_on_change);
    ESP_LOGI(TAG, "ready: B=%" PRId32 " C=%" PRId32 " S=%" PRId32 " W=%d",
             s_value[0], s_value[1], s_value[2], s_warmth);
    return ESP_OK;
}

void camera_ui_on_touch(uint16_t x, uint16_t y)
{
    if (!s_queue || y < CAMERA_UI_Y || y >= CAMERA_UI_Y + CAMERA_UI_HEIGHT || x >= 1280) return;
    int button = (int)x * UI_BUTTONS / 1280;
    if (xQueueSend(s_queue, &button, 0) != pdTRUE) {
        ESP_LOGW(TAG, "busy, tap %d dropped", button);
    }
}

const video_frame_t *camera_ui_acquire_frame(void)
{
    if (!s_lock) return NULL;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    return &s_frame;
}

void camera_ui_release_frame(void)
{
    xSemaphoreGive(s_lock);
}
