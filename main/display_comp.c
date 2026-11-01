#include <string.h>
#include "driver/i2c_master.h"
#include "driver/ppa.h"
#include "esp_check.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_async_memcpy.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "bsp/display.h"
#include "bsp/m5stack_tab5.h"
#include "app_config.h"
#include "display_comp.h"
#include "camera_ui.h"

static const char *TAG = "display_comp";

/* Logical (landscape) canvas, split into two panes. */
#define CANVAS_W    (APP_LCD_PHYS_H)
#define CANVAS_H    (APP_LCD_PHYS_W)
#define PANE_W      (CANVAS_W / DISP_PANE_COUNT)
#define PANE_H      (CAMERA_UI_Y)
#define FB_BYTES    (APP_LCD_PHYS_W * APP_LCD_PHYS_H * 2)

#define PPA_SCALE_FRAC  16      /* PPA scale resolution: 1/16 */

#define EVT_PANE_BITS   ((1 << DISP_PANE_COUNT) - 1)
#define EVT_REDRAW_BIT  (1 << DISP_PANE_COUNT)

#define ST712X_TOUCH_ADDR   0x55    /* used only to tell the LCD variants apart (same as the BSP) */

typedef struct { int x, y, w, h; } rect_t;

typedef enum { DRAWN_UNKNOWN = 0, DRAWN_BLACK, DRAWN_NO_SIGNAL, DRAWN_LIVE } drawn_state_t;

typedef struct {
    drawn_state_t state;
    rect_t rect;            /* image rect (physical) when LIVE */
} drawn_t;

typedef struct {
    frame_mbox_t mbox;
    int rot_deg;
    uint16_t no_signal;
    bool has_cur;
    video_frame_t cur;
    int64_t last_us;
} pane_t;

static esp_lcd_panel_handle_t s_panel;
static uint8_t *s_fb[2];
static ppa_client_handle_t s_srm, s_fill;
static EventGroupHandle_t s_evt;
static SemaphoreHandle_t s_flip_sem;
static pane_t s_panes[DISP_PANE_COUNT];
static drawn_t s_drawn[2][DISP_PANE_COUNT];
static display_comp_stats_t s_stats;

/* ---- geometry ------------------------------------------------------------ */

/* Logical landscape rect -> physical portrait rect (canvas rotated CCW by APP_DISPLAY_ROT_DEG). */
static rect_t logical_to_phys(rect_t r)
{
#if APP_DISPLAY_ROT_DEG == 90
    return (rect_t){ .x = r.y, .y = CANVAS_W - (r.x + r.w), .w = r.h, .h = r.w };
#elif APP_DISPLAY_ROT_DEG == 270
    return (rect_t){ .x = CANVAS_H - (r.y + r.h), .y = r.x, .w = r.h, .h = r.w };
#else
#error "APP_DISPLAY_ROT_DEG must be 90 or 270"
#endif
}

static ppa_srm_rotation_angle_t rot_to_ppa(int deg)
{
    switch (deg % 360) {
    case 90:  return PPA_SRM_ROTATION_ANGLE_90;
    case 180: return PPA_SRM_ROTATION_ANGLE_180;
    case 270: return PPA_SRM_ROTATION_ANGLE_270;
    default:  return PPA_SRM_ROTATION_ANGLE_0;
    }
}

/* Aspect-fit the (already rotated) logical image into a pane. Returns the PPA
 * scale in 1/16 units and the logical image rect. */
static void fit_image(const video_frame_t *f, int src_rot, int pane, int *scale16, rect_t *logical)
{
    int lw = (src_rot % 180) ? f->h : f->w;
    int lh = (src_rot % 180) ? f->w : f->h;
    int n = PPA_SCALE_FRAC * PANE_W / lw;
    int nh = PPA_SCALE_FRAC * PANE_H / lh;
    if (nh < n) {
        n = nh;
    }
    if (n < 1) {
        n = 1;
    }
    if (n > 255) {
        n = 255;
    }
    int dw = lw * n / PPA_SCALE_FRAC;   /* same integer maths as the PPA driver */
    int dh = lh * n / PPA_SCALE_FRAC;
    *scale16 = n;
    logical->w = dw;
    logical->h = dh;
    logical->x = (pane * PANE_W + (PANE_W - dw) / 2) & ~1;
    logical->y = ((PANE_H - dh) / 2) & ~1;
}

/* ---- PPA helpers ---------------------------------------------------------- */

static esp_err_t fill_rect(uint8_t *fb, rect_t r, uint16_t rgb565)
{
    ppa_fill_oper_config_t cfg = {
        .out = {
            .buffer = fb,
            .buffer_size = FB_BYTES,
            .pic_w = APP_LCD_PHYS_W,
            .pic_h = APP_LCD_PHYS_H,
            .block_offset_x = r.x,
            .block_offset_y = r.y,
            .fill_cm = PPA_FILL_COLOR_MODE_RGB565,
        },
        .fill_block_w = r.w,
        .fill_block_h = r.h,
        .fill_argb_color = {
            .a = 255,
            .r = ((rgb565 >> 11) & 0x1f) << 3,
            .g = ((rgb565 >> 5) & 0x3f) << 2,
            .b = (rgb565 & 0x1f) << 3,
        },
        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    int64_t t0 = esp_timer_get_time();
    esp_err_t err = ppa_do_fill(s_fill, &cfg);
    s_stats.fill_us += (uint32_t)(esp_timer_get_time() - t0);
    s_stats.fill_calls++;
    return err;
}

static esp_err_t draw_image(uint8_t *fb, const video_frame_t *f, int total_rot, int scale16, rect_t phys)
{
    float scale = (float)scale16 / PPA_SCALE_FRAC;
    ppa_srm_oper_config_t cfg = {
        .in = {
            .buffer = f->data,
            .pic_w = f->stride,
            .pic_h = f->h,
            .block_w = f->w,
            .block_h = f->h,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer = fb,
            .buffer_size = FB_BYTES,
            .pic_w = APP_LCD_PHYS_W,
            .pic_h = APP_LCD_PHYS_H,
            .block_offset_x = phys.x,
            .block_offset_y = phys.y,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .rotation_angle = rot_to_ppa(total_rot),
        .scale_x = scale,
        .scale_y = scale,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    int64_t t0 = esp_timer_get_time();
    esp_err_t err = ppa_do_scale_rotate_mirror(s_srm, &cfg);
    s_stats.srm_us += (uint32_t)(esp_timer_get_time() - t0);
    s_stats.srm_calls++;
    return err;
}

/* ---- composition ---------------------------------------------------------- */

static bool rect_eq(rect_t a, rect_t b)
{
    return a.x == b.x && a.y == b.y && a.w == b.w && a.h == b.h;
}

static void draw_pane(int fb_i, int p)
{
    pane_t *pn = &s_panes[p];
    uint8_t *fb = s_fb[fb_i];
    drawn_t *d = &s_drawn[fb_i][p];
    rect_t pane_phys = logical_to_phys((rect_t){ p * PANE_W, 0, PANE_W, PANE_H });

    if (!pn->has_cur) {
        if (d->state != DRAWN_NO_SIGNAL) {
            d->state = fill_rect(fb, pane_phys, pn->no_signal) == ESP_OK ? DRAWN_NO_SIGNAL : DRAWN_UNKNOWN;
        }
        return;
    }

    int scale16;
    rect_t logical;
    fit_image(&pn->cur, pn->rot_deg, p, &scale16, &logical);
    rect_t img_phys = logical_to_phys(logical);

    if (d->state != DRAWN_LIVE || !rect_eq(d->rect, img_phys)) {
        fill_rect(fb, pane_phys, 0);    /* clear letterbox / previous content */
    }
    esp_err_t err = draw_image(fb, &pn->cur, APP_DISPLAY_ROT_DEG + pn->rot_deg, scale16, img_phys);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "pane %d: PPA SRM failed (%s)", p, esp_err_to_name(err));
        d->state = DRAWN_UNKNOWN;
        return;
    }
    d->state = DRAWN_LIVE;
    d->rect = img_phys;
}

static bool IRAM_ATTR on_frame_buf_complete(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_flip_sem, &woken);
    return woken == pdTRUE;
}

static void display_task(void *arg)
{
    int back = 1;
    bool dirty = true;      /* draw the initial "no signal" panes */

    for (;;) {
        EventBits_t bits = xEventGroupWaitBits(s_evt, EVT_PANE_BITS | EVT_REDRAW_BIT, pdTRUE, pdFALSE,
                                               pdMS_TO_TICKS(200));
        int64_t now = esp_timer_get_time();
        if (bits & EVT_REDRAW_BIT) {
            dirty = true;
        }

        for (int p = 0; p < DISP_PANE_COUNT; p++) {
            pane_t *pn = &s_panes[p];
            if (!pn->mbox) {
                continue;
            }
            video_frame_t f;
            if ((bits & (1 << p)) && frame_mbox_take(pn->mbox, &f)) {
                if (pn->has_cur) {
                    frame_mbox_release(pn->mbox, &pn->cur);
                }
                pn->cur = f;
                pn->has_cur = true;
                pn->last_us = now;
                dirty = true;
            } else if (pn->has_cur && (now - pn->last_us) > APP_PANE_STALE_MS * 1000LL) {
                frame_mbox_release(pn->mbox, &pn->cur);
                pn->has_cur = false;
                dirty = true;
            }
        }
        if (!dirty) {
            continue;
        }
        dirty = false;

        for (int p = 0; p < DISP_PANE_COUNT; p++) {
            draw_pane(back, p);
        }
        const video_frame_t *ui = camera_ui_acquire_frame();
        if (ui) {
            rect_t ui_logical = { 0, CAMERA_UI_Y, CANVAS_W, CAMERA_UI_HEIGHT };
            draw_image(s_fb[back], ui, APP_DISPLAY_ROT_DEG, 2 * PPA_SCALE_FRAC,
                       logical_to_phys(ui_logical));
            camera_ui_release_frame();
        }

        /* Flip, then wait until the old buffer has left the scan-out. */
        xSemaphoreTake(s_flip_sem, 0);
        int64_t t_flip = esp_timer_get_time();
        esp_err_t err = esp_lcd_panel_draw_bitmap(s_panel, 0, 0, APP_LCD_PHYS_W, APP_LCD_PHYS_H, s_fb[back]);
        int64_t t_wait = esp_timer_get_time();
        s_stats.flip_call_us += (uint32_t)(t_wait - t_flip);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "flip failed (%s)", esp_err_to_name(err));
            continue;
        }
        if (xSemaphoreTake(s_flip_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "flip: frame-complete timeout");
            /* Do not reuse either framebuffer until scan-out completion is
             * confirmed. Toggling here could make the next draw overwrite the
             * buffer that the panel is still reading. */
            xSemaphoreTake(s_flip_sem, portMAX_DELAY);
        }
        s_stats.flip_wait_us += (uint32_t)(esp_timer_get_time() - t_wait);
        back ^= 1;
        s_stats.flips++;
    }
}

/* ---- init ------------------------------------------------------------------ */

/* The ST7121 LCD variant needs a lower DSI lane rate. The BSP detects it by
 * probing the touch controller; its detection helper is private, so repeat it. */
static uint32_t detect_dsi_lane_mbps(void)
{
    uint32_t mbps = BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS;

    if (bsp_i2c_init() != ESP_OK || bsp_feature_enable(BSP_FEATURE_TOUCH, true) != ESP_OK) {
        return mbps;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (i2c_master_probe(bus, ST712X_TOUCH_ADDR, 100) != ESP_OK) {
        return mbps;    /* ILI9881C + GT911 board */
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ST712X_TOUCH_ADDR,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev;
    if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) != ESP_OK) {
        return mbps;
    }
    const uint8_t reg[2] = { 0x00, 0x00 };
    uint8_t fw = 0;
    if (i2c_master_transmit_receive(dev, reg, sizeof(reg), &fw, 1, 100) == ESP_OK && fw == 1) {
        mbps = 965;     /* ST7121 */
    }
    i2c_master_bus_rm_device(dev);
    ESP_LOGI(TAG, "ST712x touch fw=%u -> DSI lane %u Mbps", fw, (unsigned)mbps);
    return mbps;
}

esp_err_t display_comp_init(void)
{
    s_evt = xEventGroupCreate();
    s_flip_sem = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(s_evt && s_flip_sem, ESP_ERR_NO_MEM, TAG, "no memory");

    bsp_display_config_t cfg = {
        .dsi_bus = {
            .phy_clk_src = 0,
            .lane_bit_rate_mbps = detect_dsi_lane_mbps(),
        },
    };
    esp_lcd_panel_io_handle_t io;
    ESP_RETURN_ON_ERROR(bsp_display_new(&cfg, &s_panel, &io), TAG, "bsp_display_new failed");

    void *fb0 = NULL, *fb1 = NULL;
    ESP_RETURN_ON_ERROR(esp_lcd_dpi_panel_get_frame_buffer(s_panel, 2, &fb0, &fb1), TAG, "get frame buffers failed");
    s_fb[0] = fb0;
    s_fb[1] = fb1;

    const ppa_client_config_t srm_cfg = { .oper_type = PPA_OPERATION_SRM, .max_pending_trans_num = 1 };
    const ppa_client_config_t fill_cfg = { .oper_type = PPA_OPERATION_FILL, .max_pending_trans_num = 1 };
    ESP_RETURN_ON_ERROR(ppa_register_client(&srm_cfg, &s_srm), TAG, "PPA SRM client failed");
    ESP_RETURN_ON_ERROR(ppa_register_client(&fill_cfg, &s_fill), TAG, "PPA fill client failed");

    const rect_t whole = { 0, 0, APP_LCD_PHYS_W, APP_LCD_PHYS_H };
    for (int i = 0; i < 2; i++) {
        ESP_RETURN_ON_ERROR(fill_rect(s_fb[i], whole, 0), TAG, "initial clear failed");
        for (int p = 0; p < DISP_PANE_COUNT; p++) {
            s_drawn[i][p].state = DRAWN_BLACK;
        }
    }

    const esp_lcd_dpi_panel_event_callbacks_t cbs = { .on_frame_buf_complete = on_frame_buf_complete };
    ESP_RETURN_ON_ERROR(esp_lcd_dpi_panel_register_event_callbacks(s_panel, &cbs, NULL), TAG, "register cb failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "panel on failed");
    ESP_RETURN_ON_ERROR(bsp_display_backlight_on(), TAG, "backlight on failed");
    ESP_LOGI(TAG, "panel %dx%d, fb0=%p fb1=%p, landscape rot=%d", APP_LCD_PHYS_W, APP_LCD_PHYS_H,
             s_fb[0], s_fb[1], APP_DISPLAY_ROT_DEG);
    return ESP_OK;
}

esp_err_t display_comp_attach(disp_pane_t pane, frame_mbox_t mbox, int rot_deg, uint16_t no_signal_rgb565)
{
    ESP_RETURN_ON_FALSE(pane < DISP_PANE_COUNT && mbox, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    s_panes[pane] = (pane_t){ .mbox = mbox, .rot_deg = rot_deg % 360, .no_signal = no_signal_rgb565 };
    frame_mbox_set_notify(mbox, s_evt, 1 << pane);
    return ESP_OK;
}


#if APP_DISPLAY_BENCH
/* One-off PPA benchmark (profiling aid): where does the SRM time go? */
static void bench_case_ex(const char *name, uint8_t *src, int sw, int sh, int rot, float scale, rect_t out_phys, uint8_t *dst,
                          int out_w, int out_h)
{
    ppa_srm_oper_config_t cfg = {
        .in = { .buffer = src, .pic_w = sw, .pic_h = sh, .block_w = sw, .block_h = sh, .srm_cm = PPA_SRM_COLOR_MODE_RGB565 },
        .out = { .buffer = dst, .buffer_size = out_w * out_h * 2, .pic_w = out_w, .pic_h = out_h,
                 .block_offset_x = out_phys.x, .block_offset_y = out_phys.y, .srm_cm = PPA_SRM_COLOR_MODE_RGB565 },
        .rotation_angle = rot_to_ppa(rot), .scale_x = scale, .scale_y = scale, .mode = PPA_TRANS_MODE_BLOCKING,
    };
    int64_t t0 = esp_timer_get_time();
    esp_err_t err = ESP_OK;
    for (int i = 0; i < 5 && err == ESP_OK; i++) {
        err = ppa_do_scale_rotate_mirror(s_srm, &cfg);
    }
    ESP_LOGI(TAG, "bench %-34s %6d us/call (%s)", name, (int)((esp_timer_get_time() - t0) / 5), esp_err_to_name(err));
}

static void bench_case(const char *name, uint8_t *src, int sw, int sh, int rot, float scale, rect_t out_phys, uint8_t *dst)
{
    bench_case_ex(name, src, sw, sh, rot, scale, out_phys, dst, APP_LCD_PHYS_W, APP_LCD_PHYS_H);
}


static bool IRAM_ATTR bench_copy_done(async_memcpy_handle_t mcp, async_memcpy_event_t *event, void *arg)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)arg, &woken);
    return woken == pdTRUE;
}

static void bench_dma_copy(void)
{
    const size_t n = 128 * 1024;
    uint8_t *ps = heap_caps_aligned_calloc(64, 1, n, MALLOC_CAP_SPIRAM);
    uint8_t *in = heap_caps_aligned_calloc(64, 1, n, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    SemaphoreHandle_t sem = xSemaphoreCreateBinary();
    if (!ps || !in || !sem) {
        ESP_LOGW(TAG, "bench dma copy: no memory");
        return;
    }
    static const size_t bursts[] = { 0, 16, 32, 64, 128 };
    for (size_t b = 0; b < sizeof(bursts) / sizeof(bursts[0]); b++) {
        async_memcpy_handle_t mcp = NULL;
        async_memcpy_config_t cfg = ASYNC_MEMCPY_DEFAULT_CONFIG();
        cfg.dma_burst_size = bursts[b];
        esp_err_t err = esp_async_memcpy_install_gdma_axi(&cfg, &mcp);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "bench dma burst %u: install failed (%s)", (unsigned)bursts[b], esp_err_to_name(err));
            continue;
        }
        int64_t t0 = esp_timer_get_time();
        for (int i = 0; i < 10; i++) {
            err = esp_async_memcpy(mcp, in, ps, n, bench_copy_done, sem);
            if (err != ESP_OK || xSemaphoreTake(sem, pdMS_TO_TICKS(100)) != pdTRUE) {
                break;
            }
        }
        int64_t dt = esp_timer_get_time() - t0;
        ESP_LOGI(TAG, "bench AXI-GDMA burst %3u psram->internal 128KB: %d us (%d MB/s) (%s)", (unsigned)bursts[b],
                 (int)(dt / 10), (int)(10.0 * n / dt), esp_err_to_name(err));
        esp_async_memcpy_uninstall(mcp);
    }
}

static void run_bench(void)
{
    bench_dma_copy();
    uint8_t *src = heap_caps_aligned_calloc(64, 1, 1280 * 720 * 2, MALLOC_CAP_SPIRAM);
    uint8_t *isrc = heap_caps_aligned_calloc(64, 1, 320 * 180 * 2, MALLOC_CAP_INTERNAL);
    if (!src || !isrc) {
        ESP_LOGW(TAG, "bench: no memory");
        free(src);
        free(isrc);
        return;
    }
    uint8_t *fb = s_fb[1];

    int64_t t0 = esp_timer_get_time();
    esp_err_t e1 = esp_cache_msync(src, 1280 * 720 * 2, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    int64_t t1 = esp_timer_get_time();
    esp_err_t e2 = esp_cache_msync(fb, 640 * 1440, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    int64_t t2 = esp_timer_get_time();
    esp_err_t e3 = esp_cache_msync(src, 1280 * 720 * 2, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    int64_t t3 = esp_timer_get_time();
    ESP_LOGI(TAG, "bench msync C2M 1.8MB: %d us (%s), again: %d us (%s); M2C 0.9MB: %d us (%s)",
             (int)(t1 - t0), esp_err_to_name(e1), (int)(t3 - t2), esp_err_to_name(e3), (int)(t2 - t1), esp_err_to_name(e2));
    bench_case("1280x720 rot90 x0.5 (current)",   src, 1280, 720, 90, 0.5f, (rect_t){ 0, 0, 360, 640 }, fb);
    bench_case("1280x720 rot0  x0.5",             src, 1280, 720, 0,  0.5f, (rect_t){ 0, 0, 640, 360 }, fb);
    bench_case("1280x720 rot90 x1.0 (no scale)",  src, 1280, 720, 90, 1.0f, (rect_t){ 0, 0, 720, 1280 }, fb);
    bench_case("1280x720 rot0  x1.0 (copy)",      src, 1280, 720, 0,  1.0f, (rect_t){ 0, 0, 720, 720 }, fb);
    bench_case("640x480 rot90 x0.75",             src, 640, 480, 90, 0.75f, (rect_t){ 0, 0, 360, 480 }, fb);
    bench_case("640x360 rot90 x1.0",              src, 640, 360, 90, 1.0f, (rect_t){ 0, 0, 360, 640 }, fb);
    bench_case("320x180 internal rot90 x1.0",     isrc, 320, 180, 90, 1.0f, (rect_t){ 0, 0, 180, 320 }, fb);

    /* where does the time go: source memory, destination memory, or PSRAM bandwidth in general? */
    uint8_t *psrc = heap_caps_aligned_calloc(64, 1, 640 * 360 * 2, MALLOC_CAP_SPIRAM);
    uint8_t *idst = heap_caps_aligned_calloc(64, 1, 360 * 640 * 2, MALLOC_CAP_SPIRAM);
    uint8_t *sdst = heap_caps_aligned_calloc(64, 1, 180 * 320 * 2, MALLOC_CAP_INTERNAL);
    uint8_t *ssrc = heap_caps_aligned_calloc(64, 1, 320 * 180 * 2, MALLOC_CAP_INTERNAL);
    if (psrc && idst && sdst && ssrc) {
        bench_case_ex("psram 640x360 -> psram own buf", psrc, 640, 360, 90, 1.0f, (rect_t){ 0, 0, 360, 640 }, idst, 360, 640);
        bench_case_ex("psram 640x360 x0.5 -> internal", psrc, 640, 360, 90, 0.5f, (rect_t){ 0, 0, 180, 320 }, sdst, 180, 320);
        bench_case_ex("internal 320x180 -> internal", ssrc, 320, 180, 90, 1.0f, (rect_t){ 0, 0, 180, 320 }, sdst, 180, 320);
        bench_case_ex("internal 320x180 -> psram own buf", ssrc, 320, 180, 90, 1.0f, (rect_t){ 0, 0, 180, 320 }, idst, 360, 640);
        int64_t t0 = esp_timer_get_time();
        for (int i = 0; i < 5; i++) {
            memcpy(idst, psrc, 640 * 360 * 2);
        }
        ESP_LOGI(TAG, "bench CPU memcpy psram->psram 460KB: %d us (%d MB/s)", (int)((esp_timer_get_time() - t0) / 5),
                 (int)(640 * 360 * 2 * 5 / (double)(esp_timer_get_time() - t0)));
    }
    free(psrc); free(idst); free(sdst); free(ssrc);
    free(src);
    free(isrc);
}
#endif

esp_err_t display_comp_start(void)
{
#if APP_DISPLAY_BENCH
    run_bench();
#endif
    ESP_RETURN_ON_FALSE(xTaskCreate(display_task, "display", APP_TASK_DISPLAY_STACK, NULL,
                                    APP_TASK_DISPLAY_PRIO, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task create failed");
    return ESP_OK;
}

void display_comp_request_redraw(void)
{
    if (s_evt) {
        xEventGroupSetBits(s_evt, EVT_REDRAW_BIT);
    }
}

void display_comp_get_stats(display_comp_stats_t *out)
{
    *out = s_stats;
}
