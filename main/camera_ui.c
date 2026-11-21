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
#include "cam_uvc.h"
#include "cam_uvc_ctrl.h"
#include "camera_ui.h"
#include "ui_draw.h"
#include "measure_ui.h"
#include "display_comp.h"
#include "recorder.h"
#include "net_wifi.h"

static const char *TAG = "camera_ui";

#define UI_SRC_W       640
#define UI_SRC_H        40
#define UI_BUTTONS      12      /* target, 4 x (-, +), reset, Wi-Fi, record */
#define UI_GROUPS        4      /* B, C, S, W: each a "-" / "+" pair */
#define UI_BTN_TARGET    0
#define UI_BTN_RESET     9
#define UI_BTN_WIFI     10
#define UI_BTN_REC      11
#define UI_REC_POLL_MS 500      /* recording and Wi-Fi state, elapsed time refresh */
#define UI_FONT_SCALE    2
#define UI_LABEL_Y       4
#define UI_VALUE_Y      22
#define UI_CENTER_Y     13
#define UI_NONE         (-1)
#define UI_EVT_UVC      100     /* queue item: UVC controls appeared or vanished */
#define UI_EVT_TAB      200     /* queue item: + page index */
#define UI_EVT_SRC_IN   300     /* settings page: record the built-in camera */
#define UI_EVT_SRC_UVC  301     /* settings page: record the UVC camera */
#define UI_EVT_PAGE_TAP 1000000 /* measure page: + x * 1000 + y (page image coordinates) */

/* Tabs and pages are drawn at half resolution like the bar (the display scales by 2). */
#define UI_TAB_SRC_H    (CAMERA_UI_TAB_HEIGHT / 2)
#define UI_PAGE_SRC_H   ((720 - CAMERA_UI_TAB_HEIGHT) / 2)
/* Settings page: the two recording-source buttons (page image coordinates). */
#define SET_BTN_Y       56
#define SET_BTN_H       84
#define SET_BTN_IN_X    20
#define SET_BTN_UVC_X   330
#define SET_BTN_W       290

#define COLOR_PAGE_BG   0x0841
#define COLOR_TAB_BG    0x10a2
#define COLOR_TAB_ON    0x2b5f  /* selected tab */
#define COLOR_OPT       0x2104
#define COLOR_OPT_ON    0x03a6  /* selected option (green) */
#define COLOR_DIM       0x8410

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
#define COLOR_WIFI_OFF  0x2104
#define COLOR_WIFI_ON   0x0270  /* teal: Wi-Fi up */

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
/* Tab strip and page image (PSRAM), double-buffered the same way under s_lock. */
static uint16_t *s_tab_buf[2], *s_page_buf[2];
static int s_tab_front, s_page_front;
static video_frame_t s_tab_frame, s_page_frame;
static volatile camera_ui_page_t s_page = CAMERA_UI_PAGE_CAMERA;
static const char *const s_tab_labels[CAMERA_UI_PAGE_COUNT] = { "CAMERA", "MEASURE", "SETTINGS" };
static cam_uvc_stats_t s_uvc_seen;      /* UVC state as last drawn on the settings page */
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
static net_status_t s_net;          /* as last drawn */

static const char *const s_labels[UI_BUTTONS] = {
    "", "B-", "B+", "C-", "C+", "S-", "S+", "W-", "W+", "RST", "WIFI", "REC"
};

/* Drawing (ui_draw.c) at the bar's font scale. */
static inline void put_rect(canvas_t *c, int x, int y, int w, int h, uint16_t color)
{
    ui_rect(c, x, y, w, h, color);
}

static inline void put_text(canvas_t *c, int x, int y, const char *text, uint16_t color)
{
    ui_text(c, x, y, text, color, UI_FONT_SCALE);
}

static inline void put_text_centered(canvas_t *c, int x0, int x1, int y, const char *text, uint16_t color)
{
    ui_text_centered(c, x0, x1, y, text, color, UI_FONT_SCALE);
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
static void render_rec(canvas_t *c)
{
    int x0 = button_x(UI_BTN_REC);
    uint16_t bg = s_pressed == UI_BTN_REC ? COLOR_PRESSED
                : s_rec.state == RECORDER_RECORDING ? COLOR_REC_ON
                : s_rec.state == RECORDER_IDLE ? COLOR_REC_IDLE : COLOR_REC_FAIL;
    put_rect(c, x0 + 1, 1, UI_SRC_W - x0 - 2, UI_SRC_H - 2, bg);
    put_text_centered(c, x0, UI_SRC_W, UI_LABEL_Y, s_labels[UI_BTN_REC], COLOR_LABEL);

    char text[8] = "";
    if (s_rec.state == RECORDER_RECORDING) {
        snprintf(text, sizeof(text), "%" PRIu32 ":%02" PRIu32, s_rec.seconds / 60 % 100, s_rec.seconds % 60);
    } else if (s_rec.state == RECORDER_IDLE) {
        snprintf(text, sizeof(text), "%s", s_rec.source == RECORDER_SRC_UVC ? "UVC" : "IN");
    } else if (s_rec.state == RECORDER_NO_CARD) {
        snprintf(text, sizeof(text), "NOSD");
    } else if (s_rec.state == RECORDER_FULL) {
        snprintf(text, sizeof(text), "FULL");
    } else if (s_rec.state == RECORDER_ERROR) {
        snprintf(text, sizeof(text), "ERR");
    }
    put_text_centered(c, x0, UI_SRC_W, UI_VALUE_Y, text, COLOR_VALUE);
}

/* WIFI: teal while on, with AP (access point only) or STA (also joined the
 * saved network) underneath; ... while starting, ERR when the C6 failed. */
static void render_wifi(canvas_t *c)
{
    int x0 = button_x(UI_BTN_WIFI), x1 = button_x(UI_BTN_WIFI + 1);
    uint16_t bg = s_pressed == UI_BTN_WIFI ? COLOR_PRESSED
                : s_net.state == NET_ON ? COLOR_WIFI_ON
                : s_net.state == NET_FAILED ? COLOR_REC_FAIL : COLOR_WIFI_OFF;
    put_rect(c, x0 + 1, 1, x1 - x0 - 2, UI_SRC_H - 2, bg);
    put_text_centered(c, x0, x1, UI_LABEL_Y, s_labels[UI_BTN_WIFI], COLOR_LABEL);
    const char *text = s_net.state == NET_STARTING ? "..."
                     : s_net.state == NET_FAILED ? "ERR"
                     : s_net.state == NET_OFF ? "OFF"
                     : s_net.sta_connected ? "STA" : "AP";
    put_text_centered(c, x0, x1, UI_VALUE_Y, text, COLOR_VALUE);
}

/* Target switch, then each group drawn as one stepper ("-" and "+" labels on
 * top, the current value centred across both buttons underneath), then reset
 * and record. */
static void render_bar(canvas_t *c)
{
    put_rect(c, 0, 0, UI_SRC_W, UI_SRC_H, COLOR_BAR);

    int tx1 = button_x(UI_BTN_TARGET + 1);
    uint16_t tbg = s_pressed == UI_BTN_TARGET ? COLOR_PRESSED
                 : s_target == TARGET_UVC ? COLOR_TARGET_UV : COLOR_TARGET_IN;
    put_rect(c, 1, 1, tx1 - 2, UI_SRC_H - 2, tbg);
    put_text_centered(c, 0, tx1, UI_CENTER_Y, s_target == TARGET_UVC ? "UVC" : "IN", COLOR_LABEL);

    for (int g = 0; g < UI_GROUPS; g++) {
        int bm = 1 + 2 * g, bp = bm + 1;
        int x0 = button_x(bm), xm = button_x(bp), x1 = button_x(bp + 1);
        uint16_t bg = (g & 1) ? COLOR_GROUP_B : COLOR_GROUP_A;
        put_rect(c, x0 + 1, 1, x1 - x0 - 2, UI_SRC_H - 2, bg);
        if (s_pressed == bm) put_rect(c, x0 + 1, 1, xm - x0 - 1, UI_SRC_H - 2, COLOR_PRESSED);
        if (s_pressed == bp) put_rect(c, xm, 1, x1 - xm - 1, UI_SRC_H - 2, COLOR_PRESSED);
        put_rect(c, xm, 3, 1, UI_VALUE_Y - 5, COLOR_DIVIDER);
        put_text_centered(c, x0, xm, UI_LABEL_Y, s_labels[bm], COLOR_LABEL);
        put_text_centered(c, xm, x1, UI_LABEL_Y, s_labels[bp], COLOR_LABEL);

        char text[8];
        format_value(g, text, sizeof(text));
        put_text_centered(c, x0, x1, UI_VALUE_Y, text, COLOR_VALUE);
    }

    int x0 = button_x(UI_BTN_RESET), x1 = button_x(UI_BTN_RESET + 1);
    put_rect(c, x0 + 1, 1, x1 - x0 - 2, UI_SRC_H - 2,
             s_pressed == UI_BTN_RESET ? COLOR_RESET_ON : COLOR_RESET);
    put_text_centered(c, x0, x1, UI_CENTER_Y, s_labels[UI_BTN_RESET], COLOR_LABEL);

    render_wifi(c);
    render_rec(c);
}

/* ---- tabs and pages ------------------------------------------------------------ */

static void render_tabs(canvas_t *c, camera_ui_page_t active)
{
    put_rect(c, 0, 0, c->w, c->h, COLOR_TAB_BG);
    for (int i = 0; i < CAMERA_UI_PAGE_COUNT; i++) {
        int x0 = i * c->w / CAMERA_UI_PAGE_COUNT, x1 = (i + 1) * c->w / CAMERA_UI_PAGE_COUNT;
        if (i == (int)active) {
            put_rect(c, x0 + 1, 1, x1 - x0 - 2, c->h - 2, COLOR_TAB_ON);
        }
        put_text_centered(c, x0, x1, (c->h - 7 * UI_FONT_SCALE) / 2, s_tab_labels[i],
                          i == (int)active ? COLOR_LABEL : COLOR_DIM);
    }
}

/* One selectable option: a box with a title and a detail line. */
static void render_option(canvas_t *c, int x, const char *title, const char *detail, bool on, bool enabled)
{
    put_rect(c, x, SET_BTN_Y, SET_BTN_W, SET_BTN_H, on ? COLOR_LABEL : COLOR_DIVIDER);
    put_rect(c, x + 3, SET_BTN_Y + 3, SET_BTN_W - 6, SET_BTN_H - 6, on ? COLOR_OPT_ON : COLOR_OPT);
    uint16_t fg = enabled ? COLOR_LABEL : COLOR_DIM;
    put_text_centered(c, x, x + SET_BTN_W, SET_BTN_Y + 18, title, fg);
    put_text_centered(c, x, x + SET_BTN_W, SET_BTN_Y + 50, detail, on ? COLOR_VALUE : fg);
}

static void render_settings(canvas_t *c)
{
    put_rect(c, 0, 0, c->w, c->h, COLOR_PAGE_BG);
    put_text(c, 20, 24, "RECORD CAMERA", COLOR_LABEL);
    const bool uvc_rec = s_rec.source == RECORDER_SRC_UVC;
    const bool locked = s_rec.state == RECORDER_RECORDING;
    char uvc_detail[24];
    if (s_uvc_seen.streaming) {
        snprintf(uvc_detail, sizeof(uvc_detail), "%uX%u", s_uvc_seen.w, s_uvc_seen.h);
    } else {
        snprintf(uvc_detail, sizeof(uvc_detail), "NOT CONNECTED");
    }
    char in_detail[24];
    snprintf(in_detail, sizeof(in_detail), "%dX%d", APP_CAM_BUILTIN_W, APP_CAM_BUILTIN_H);
    render_option(c, SET_BTN_IN_X, "BUILT-IN", in_detail, !uvc_rec, !locked);
    render_option(c, SET_BTN_UVC_X, "UVC", uvc_detail, uvc_rec, !locked && s_uvc_seen.streaming);
    if (locked) {
        put_text(c, 20, SET_BTN_Y + SET_BTN_H + 20, "STOP RECORDING TO CHANGE", COLOR_VALUE);
    }
}

/* Render the tabs and (off the camera page) the page into the back images.
 * The page switches together with its images, under s_lock: the display never
 * sees the new page with the previous page's image. */
static void publish_page(camera_ui_page_t page_id)
{
    int tb = s_tab_front ^ 1, pb = s_page_front ^ 1;
    canvas_t tabs = { s_tab_buf[tb], UI_SRC_W, UI_TAB_SRC_H };
    render_tabs(&tabs, page_id);
    canvas_t page = { s_page_buf[pb], UI_SRC_W, UI_PAGE_SRC_H };
    if (page_id == CAMERA_UI_PAGE_SETTINGS) {
        render_settings(&page);
    } else if (page_id == CAMERA_UI_PAGE_MEASURE) {
        measure_ui_render(&page);
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_page = page_id;
    s_tab_front = tb;
    s_tab_frame.data = (uint8_t *)s_tab_buf[tb];
    if (page_id != CAMERA_UI_PAGE_CAMERA) {
        s_page_front = pb;
        s_page_frame.data = (uint8_t *)s_page_buf[pb];
    }
    xSemaphoreGive(s_lock);
    display_comp_request_redraw();
}

/* Render into the back image, swap it in and ask the display for a redraw. */
static void publish_bar(void)
{
    int back = s_front ^ 1;
    int64_t t0 = esp_timer_get_time();
    canvas_t bar = { s_buf[back], UI_SRC_W, UI_SRC_H };
    render_bar(&bar);
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
    /* Auto first: while it is on, the camera refuses its manual partner. */
    if (s_uvc_auto_ok) {
        cam_uvc_ctrl_set(CAM_UVC_CTRL_WB_TEMP_AUTO, s_uvc_auto.default_value);
    }
    for (int g = 0; g < UI_GROUPS - 1; g++) {
        if (s_uvc_ok[g]) {
            cam_uvc_ctrl_set(s_uvc_ctrl[g], s_uvc[g].default_value);
        }
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
    if (button == UI_BTN_WIFI) {
        bool on = s_net.state == NET_ON || s_net.state == NET_STARTING;
        net_wifi_enable(!on);
        ESP_LOGI(TAG, "Wi-Fi %s requested", on ? "off" : "on");
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
            if (button >= UI_EVT_TAB && button < UI_EVT_TAB + CAMERA_UI_PAGE_COUNT) {
                const camera_ui_page_t page = (camera_ui_page_t)(button - UI_EVT_TAB);
                recorder_get_stats(&s_rec);
                cam_uvc_get_stats(&s_uvc_seen);
                ESP_LOGI(TAG, "page -> %s", s_tab_labels[page]);
                publish_page(page);
                publish_bar();
                continue;
            }
            if (button >= UI_EVT_PAGE_TAP) {
                int v = button - UI_EVT_PAGE_TAP;
                if (s_page == CAMERA_UI_PAGE_MEASURE && measure_ui_tap(v / 1000, v % 1000)) {
                    publish_page(s_page);
                }
                continue;
            }
            if (button == UI_EVT_SRC_IN || button == UI_EVT_SRC_UVC) {
                recorder_source_t src = button == UI_EVT_SRC_UVC ? RECORDER_SRC_UVC : RECORDER_SRC_BUILTIN;
                esp_err_t err = (src == RECORDER_SRC_UVC && !s_uvc_seen.streaming) ? ESP_ERR_INVALID_STATE
                                : recorder_set_source(src);
                ESP_LOGI(TAG, "record source -> %s: %s", src == RECORDER_SRC_UVC ? "UVC" : "built-in",
                         esp_err_to_name(err));
                recorder_get_stats(&s_rec);
                publish_page(s_page);
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
        net_status_t net;
        net_wifi_get_status(&net);
        if (net.state != s_net.state || net.sta_connected != s_net.sta_connected) {
            redraw = true;
        }
        s_net = net;
        bool page_redraw = false;
        if (s_page == CAMERA_UI_PAGE_SETTINGS) {
            cam_uvc_stats_t uvc;
            cam_uvc_get_stats(&uvc);
            page_redraw = redraw || uvc.streaming != s_uvc_seen.streaming || uvc.w != s_uvc_seen.w ||
                          uvc.h != s_uvc_seen.h;
            s_uvc_seen = uvc;
        }
        if (s_page == CAMERA_UI_PAGE_MEASURE) {
            page_redraw |= measure_ui_poll();
        }
        if (page_redraw) {
            publish_page(s_page);
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
    for (int i = 0; i < 2; i++) {
        s_tab_buf[i] = heap_caps_aligned_calloc(64, UI_SRC_W * UI_TAB_SRC_H, sizeof(uint16_t), MALLOC_CAP_SPIRAM);
        s_page_buf[i] = heap_caps_aligned_calloc(64, UI_SRC_W * UI_PAGE_SRC_H, sizeof(uint16_t), MALLOC_CAP_SPIRAM);
        ESP_RETURN_ON_FALSE(s_tab_buf[i] && s_page_buf[i], ESP_ERR_NO_MEM, TAG, "no page memory");
    }
    s_lock = xSemaphoreCreateMutex();
    s_queue = xQueueCreate(8, sizeof(int));
    ESP_RETURN_ON_FALSE(s_lock && s_queue, ESP_ERR_NO_MEM, TAG, "no memory");

    recorder_get_stats(&s_rec);
    net_wifi_get_status(&s_net);
    canvas_t bar = { s_buf[0], UI_SRC_W, UI_SRC_H };
    render_bar(&bar);
    s_front = 0;
    s_frame = (video_frame_t){ .data = (uint8_t *)s_buf[0], .w = UI_SRC_W, .h = UI_SRC_H, .stride = UI_SRC_W };
    canvas_t tabs = { s_tab_buf[0], UI_SRC_W, UI_TAB_SRC_H };
    render_tabs(&tabs, s_page);
    s_tab_front = 0;
    s_tab_frame = (video_frame_t){ .data = (uint8_t *)s_tab_buf[0], .w = UI_SRC_W, .h = UI_TAB_SRC_H, .stride = UI_SRC_W };
    s_page_front = 0;
    s_page_frame = (video_frame_t){ .data = (uint8_t *)s_page_buf[0], .w = UI_SRC_W, .h = UI_PAGE_SRC_H, .stride = UI_SRC_W };
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
    if (!s_queue || x >= 1280 || y >= 720) return;
    int item = -1;
    if (y < CAMERA_UI_TAB_HEIGHT) {
        item = UI_EVT_TAB + (int)x * CAMERA_UI_PAGE_COUNT / 1280;
    } else if (s_page == CAMERA_UI_PAGE_CAMERA) {
        if (y >= CAMERA_UI_Y && y < CAMERA_UI_Y + CAMERA_UI_HEIGHT) {
            item = (int)x * UI_BUTTONS / 1280;
        }
    } else if (s_page == CAMERA_UI_PAGE_MEASURE) {
        item = UI_EVT_PAGE_TAP + (x / 2) * 1000 + (y - CAMERA_UI_TAB_HEIGHT) / 2;
    } else if (s_page == CAMERA_UI_PAGE_SETTINGS) {
        int sx = x / 2, sy = (y - CAMERA_UI_TAB_HEIGHT) / 2;
        if (sy >= SET_BTN_Y && sy < SET_BTN_Y + SET_BTN_H) {
            if (sx >= SET_BTN_IN_X && sx < SET_BTN_IN_X + SET_BTN_W) {
                item = UI_EVT_SRC_IN;
            } else if (sx >= SET_BTN_UVC_X && sx < SET_BTN_UVC_X + SET_BTN_W) {
                item = UI_EVT_SRC_UVC;
            }
        }
    }
    if (item >= 0 && xQueueSend(s_queue, &item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "busy, tap %d dropped", item);
    }
}

camera_ui_page_t camera_ui_page(void)
{
    return s_page;
}

bool camera_ui_acquire(camera_ui_frames_t *out)
{
    if (!s_lock) return false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const bool camera = s_page == CAMERA_UI_PAGE_CAMERA;
    out->tabs = &s_tab_frame;
    out->bar = camera ? &s_frame : NULL;
    out->page = camera ? NULL : &s_page_frame;
    return true;
}

void camera_ui_release(void)
{
    xSemaphoreGive(s_lock);
}
