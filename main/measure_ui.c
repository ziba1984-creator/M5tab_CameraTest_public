#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_netif_ip_addr.h"
#include "measure_ui.h"
#include "modules.h"
#include "net_eth.h"

static const char *TAG = "measure_ui";

/* Page image layout (640 x 340). */
#define LIST_X0         20
#define LIST_Y0         56
#define LIST_COLS       3
#define LIST_W          190
#define LIST_H          110
#define LIST_GAP        15
#define BACK_X          10
#define BACK_Y          8
#define BACK_W          120
#define BACK_H          40
#define FAN_ROW_Y       70
#define FAN_ROW_H       80
#define FAN_MINUS_X     20
#define FAN_VALUE_X     120
#define FAN_PLUS_X      310
#define FAN_BTN_W       90
#define FAN_ONOFF_X     430
#define FAN_ONOFF_W     190
#define FAN_STEP        10

#define NO_VIEW         (-1)    /* the module list */

static int s_view = NO_VIEW;    /* module_id_t of the open screen */

/* What the page last showed, to redraw only on change. */
typedef struct {
    int view;
    unsigned present;           /* module_id_t bits */
    fan_state_t fan;
    bool eth_link, eth_dhcp;
    uint32_t eth_ip;
} shown_t;
static shown_t s_shown;

static void snapshot(shown_t *s)
{
    memset(s, 0, sizeof(*s));
    s->view = s_view;
    for (int i = 0; i < MODULE_COUNT; i++) {
        if (module_present(i)) {
            s->present |= 1u << i;
        }
    }
    fan_get(&s->fan);
    eth_status_t eth;
    net_eth_get_status(&eth);
    s->eth_link = eth.link;
    s->eth_dhcp = eth.dhcp;
    s->eth_ip = eth.ip;
}

/* The connected modules in list order. Returns how many. */
static int list_modules(module_id_t out[MODULE_COUNT], unsigned present)
{
    int n = 0;
    for (int i = 0; i < MODULE_COUNT; i++) {
        if (present & (1u << i)) {
            out[n++] = (module_id_t)i;
        }
    }
    return n;
}

static void list_cell(int i, int *x, int *y)
{
    *x = LIST_X0 + (i % LIST_COLS) * (LIST_W + LIST_GAP);
    *y = LIST_Y0 + (i / LIST_COLS) * (LIST_H + LIST_GAP);
}

static void ip_text(char *dst, size_t size, uint32_t addr)
{
    esp_ip4_addr_t a = { .addr = addr };
    snprintf(dst, size, IPSTR, IP2STR(&a));
}

/* Second line of a module's button. */
static void module_summary(module_id_t id, const shown_t *s, char *dst, size_t size)
{
    switch (id) {
    case MODULE_LAN:
        if (!s->eth_link) {
            snprintf(dst, size, "NO CABLE");
        } else if (s->eth_ip) {
            ip_text(dst, size, s->eth_ip);
        } else {
            snprintf(dst, size, "LINK UP");
        }
        break;
    case MODULE_FAN:
        snprintf(dst, size, s->fan.enabled ? "%u%% %uRPM" : "OFF", s->fan.duty, s->fan.rpm);
        break;
    case MODULE_PPS:
        snprintf(dst, size, "FOUND");
        break;
    default:
        dst[0] = '\0';
        break;
    }
}

/* ---- rendering ------------------------------------------------------------------- */

static void render_list(canvas_t *c, const shown_t *s)
{
    ui_text(c, 20, 20, "MODULES", UI_COLOR_LABEL, 2);
    module_id_t ids[MODULE_COUNT];
    int n = list_modules(ids, s->present);
    if (n == 0) {
        ui_text_centered(c, 0, c->w, c->h / 2, "NO MODULE CONNECTED", UI_COLOR_DIM, 2);
        return;
    }
    for (int i = 0; i < n; i++) {
        int x, y;
        list_cell(i, &x, &y);
        char line2[24];
        module_summary(ids[i], s, line2, sizeof(line2));
        ui_button(c, x, y, LIST_W, LIST_H, module_name(ids[i]), line2, UI_COLOR_OPT, UI_COLOR_LABEL, 2);
    }
}

static void render_header(canvas_t *c, module_id_t id)
{
    ui_button(c, BACK_X, BACK_Y, BACK_W, BACK_H, "< BACK", NULL, UI_COLOR_OPT, UI_COLOR_LABEL, 2);
    ui_text(c, BACK_X + BACK_W + 24, BACK_Y + 13, module_name(id), UI_COLOR_LABEL, 2);
}

static void render_fan(canvas_t *c, const shown_t *s)
{
    const fan_state_t *f = &s->fan;
    if (!f->present) {
        ui_text_centered(c, 0, c->w, c->h / 2, "NOT CONNECTED", UI_COLOR_DIM, 2);
        return;
    }
    ui_button(c, FAN_MINUS_X, FAN_ROW_Y, FAN_BTN_W, FAN_ROW_H, "-", NULL, UI_COLOR_OPT, UI_COLOR_LABEL, 4);
    ui_button(c, FAN_PLUS_X, FAN_ROW_Y, FAN_BTN_W, FAN_ROW_H, "+", NULL, UI_COLOR_OPT, UI_COLOR_LABEL, 4);
    char text[24];
    snprintf(text, sizeof(text), "%u%%", f->duty);
    ui_text(c, FAN_VALUE_X + 10, FAN_ROW_Y - 2, "SPEED", UI_COLOR_DIM, 1);
    ui_text_centered(c, FAN_VALUE_X, FAN_PLUS_X, FAN_ROW_Y + 26, text, UI_COLOR_VALUE, 4);
    ui_button(c, FAN_ONOFF_X, FAN_ROW_Y, FAN_ONOFF_W, FAN_ROW_H, f->enabled ? "ON" : "OFF", NULL,
              f->enabled ? UI_COLOR_OPT_ON : UI_COLOR_OPT, UI_COLOR_LABEL, 3);
    snprintf(text, sizeof(text), "%u RPM", f->rpm);
    ui_text(c, 20, 190, text, UI_COLOR_VALUE, 4);
    static const char *const freqs[] = { "1 KHZ", "12 KHZ", "24 KHZ", "48 KHZ" };
    snprintf(text, sizeof(text), "PWM %s  FW %u", f->pwm_freq < 4 ? freqs[f->pwm_freq] : "?", f->fw);
    ui_text(c, 20, 250, text, UI_COLOR_DIM, 2);
}

static void render_lan(canvas_t *c)
{
    eth_status_t e;
    net_eth_get_status(&e);
    char ip[20], mask[20], gw[20], line[48];
    ip_text(ip, sizeof(ip), e.ip);
    ip_text(mask, sizeof(mask), e.mask);
    ip_text(gw, sizeof(gw), e.gw);
    const char *rows[6][2] = {
        { "LINK", e.link ? "UP" : "NO CABLE" },
        { "MODE", e.dhcp ? "DHCP" : "FIXED" },
        { "IP", e.ip ? ip : "-" },
        { "MASK", e.ip ? mask : "-" },
        { "GATEWAY", e.ip && e.gw ? gw : "-" },
        { "MAC", line },
    };
    snprintf(line, sizeof(line), "%02X:%02X:%02X:%02X:%02X:%02X", e.mac[0], e.mac[1], e.mac[2], e.mac[3],
             e.mac[4], e.mac[5]);
    for (int i = 0; i < 6; i++) {
        int y = 70 + i * 36;
        ui_text(c, 20, y, rows[i][0], UI_COLOR_DIM, 2);
        ui_text(c, 180, y, rows[i][1], i == 0 && !e.link ? UI_COLOR_DIM : UI_COLOR_VALUE, 2);
    }
    ui_text(c, 20, 300, "CHANGE THE ADDRESS ON THE WEB PAGE", UI_COLOR_DIM, 2);
}

static void render_pps(canvas_t *c)
{
    ui_text_centered(c, 0, c->w, 130, "CONTROL NOT AVAILABLE YET", UI_COLOR_LABEL, 2);
    ui_text_centered(c, 0, c->w, 170, "(NEEDS ITS DC 9-36V INPUT)", UI_COLOR_DIM, 2);
}

void measure_ui_render(canvas_t *c)
{
    snapshot(&s_shown);
    ui_rect(c, 0, 0, c->w, c->h, UI_COLOR_PAGE_BG);
    if (s_view == NO_VIEW) {
        render_list(c, &s_shown);
        return;
    }
    render_header(c, (module_id_t)s_view);
    switch (s_view) {
    case MODULE_FAN: render_fan(c, &s_shown); break;
    case MODULE_LAN: render_lan(c); break;
    case MODULE_PPS: render_pps(c); break;
    default: break;
    }
}

/* ---- input ------------------------------------------------------------------------ */

static bool inside(int x, int y, int x0, int y0, int w, int h)
{
    return x >= x0 && x < x0 + w && y >= y0 && y < y0 + h;
}

static bool tap_fan(int x, int y)
{
    fan_state_t f;
    fan_get(&f);
    if (!f.present || y < FAN_ROW_Y || y >= FAN_ROW_Y + FAN_ROW_H) {
        return false;
    }
    esp_err_t err = ESP_OK;
    if (inside(x, y, FAN_MINUS_X, FAN_ROW_Y, FAN_BTN_W, FAN_ROW_H)) {
        err = fan_set_duty(f.duty >= FAN_STEP ? f.duty - FAN_STEP : 0);
    } else if (inside(x, y, FAN_PLUS_X, FAN_ROW_Y, FAN_BTN_W, FAN_ROW_H)) {
        err = fan_set_duty(f.duty + FAN_STEP > 100 ? 100 : f.duty + FAN_STEP);
    } else if (inside(x, y, FAN_ONOFF_X, FAN_ROW_Y, FAN_ONOFF_W, FAN_ROW_H)) {
        err = fan_set_enabled(!f.enabled);
    } else {
        return false;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "FAN command failed (%s)", esp_err_to_name(err));
    }
    return true;
}

bool measure_ui_tap(int x, int y)
{
    if (s_view == NO_VIEW) {
        module_id_t ids[MODULE_COUNT];
        int n = list_modules(ids, s_shown.present);     /* the buttons as drawn */
        for (int i = 0; i < n; i++) {
            int cx, cy;
            list_cell(i, &cx, &cy);
            if (inside(x, y, cx, cy, LIST_W, LIST_H)) {
                s_view = ids[i];
                ESP_LOGI(TAG, "open %s", module_name(ids[i]));
                return true;
            }
        }
        return false;
    }
    if (inside(x, y, BACK_X, BACK_Y, BACK_W, BACK_H)) {
        s_view = NO_VIEW;
        return true;
    }
    return s_view == MODULE_FAN ? tap_fan(x, y) : false;
}

bool measure_ui_poll(void)
{
    shown_t now;
    snapshot(&now);
    return memcmp(&now, &s_shown, sizeof(now)) != 0;
}
