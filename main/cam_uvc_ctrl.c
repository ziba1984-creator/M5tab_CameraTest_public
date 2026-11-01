#include <inttypes.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "usb/usb_host.h"
#include "app_config.h"
#include "cam_uvc.h"
#include "cam_uvc_ctrl.h"

static const char *TAG = "cam_uvc_ctrl";

/* Declared in esp_private/uvc_control.h of usb_host_uvc, which cannot be
 * included from outside the component (it pulls in a private header). */
esp_err_t uvc_host_usb_ctrl(uvc_host_stream_hdl_t stream_hdl, uint8_t bmRequestType, uint8_t bRequest,
                            uint16_t wValue, uint16_t wIndex, uint16_t wLength, uint8_t *data);

/* USB / UVC 1.5 constants */
#define DESC_TYPE_INTERFACE     0x04
#define DESC_TYPE_CS_INTERFACE  0x24
#define USB_CLASS_VIDEO_        0x0e
#define UVC_SC_VIDEOCONTROL     0x01
#define VC_HEADER               0x01
#define VC_INPUT_TERMINAL       0x02
#define VC_PROCESSING_UNIT      0x05
#define ITT_CAMERA              0x0201
#define REQ_TYPE_GET            0xa1    /* class, interface, device-to-host */
#define REQ_TYPE_SET            0x21    /* class, interface, host-to-device */
#define REQ_SET_CUR             0x01
#define REQ_GET_CUR             0x81
#define REQ_GET_MIN             0x82
#define REQ_GET_MAX             0x83
#define REQ_GET_RES             0x84
#define REQ_GET_INFO            0x86
#define REQ_GET_DEF             0x87

#define PU_ATTEMPTS             5
#define PU_RETRY_DELAY_MS       20

typedef struct {
    const char *name;
    uint8_t bit;            /* bmControls bit of the Processing Unit descriptor */
    uint8_t selector;       /* PU control selector */
    uint8_t len;            /* value size in bytes */
    bool is_signed;
    bool is_bool;           /* auto on/off: the spec allows only GET_CUR/DEF/INFO */
    int8_t auto_ctrl;       /* auto control that overrides this one while on, or -1 */
} pu_desc_t;

static const pu_desc_t s_pu[CAM_UVC_CTRL_COUNT] = {
    [CAM_UVC_CTRL_BRIGHTNESS]    = { "brightness",    0,  0x02, 2, true,  false, -1 },
    [CAM_UVC_CTRL_CONTRAST]      = { "contrast",      1,  0x03, 2, false, false, CAM_UVC_CTRL_CONTRAST_AUTO },
    [CAM_UVC_CTRL_HUE]           = { "hue",           2,  0x06, 2, true,  false, CAM_UVC_CTRL_HUE_AUTO },
    [CAM_UVC_CTRL_SATURATION]    = { "saturation",    3,  0x07, 2, false, false, -1 },
    [CAM_UVC_CTRL_SHARPNESS]     = { "sharpness",     4,  0x08, 2, false, false, -1 },
    [CAM_UVC_CTRL_GAMMA]         = { "gamma",         5,  0x09, 2, false, false, -1 },
    [CAM_UVC_CTRL_WB_TEMP]       = { "wb_temp",       6,  0x0a, 2, false, false, CAM_UVC_CTRL_WB_TEMP_AUTO },
    [CAM_UVC_CTRL_BACKLIGHT]     = { "backlight",     8,  0x01, 2, false, false, -1 },
    [CAM_UVC_CTRL_GAIN]          = { "gain",          9,  0x04, 2, false, false, -1 },
    [CAM_UVC_CTRL_POWER_LINE]    = { "power_line",    10, 0x05, 1, false, false, -1 },
    [CAM_UVC_CTRL_HUE_AUTO]      = { "hue_auto",      11, 0x10, 1, false, true,  -1 },
    [CAM_UVC_CTRL_WB_TEMP_AUTO]  = { "wb_temp_auto",  12, 0x0b, 1, false, true,  -1 },
    [CAM_UVC_CTRL_CONTRAST_AUTO] = { "contrast_auto", 18, 0x13, 1, false, true,  -1 },
};

typedef struct {
    bool found;
    uint8_t vc_intf;
    uint8_t unit_id;
    uint32_t pu_controls;
    uint32_t ct_controls;
    uint16_t bcd_uvc;
} vc_layout_t;

/* s_lock guards everything below and is held for every control transfer, so
 * cam_uvc_ctrl_forget() (called before the stream is closed) waits for an
 * in-flight transfer instead of letting it use a freed stream handle. */
static SemaphoreHandle_t s_lock;
static uvc_host_stream_hdl_t s_stream;
static vc_layout_t s_layout;
static cam_uvc_ctrl_info_t s_info[CAM_UVC_CTRL_COUNT];
static cam_uvc_ctrl_notify_fn s_notify;

/* What the user asked for. Kept across reconnects and power cycles and applied
 * before streaming starts, which is the only time the ELP camera accepts some
 * controls (s_needs_restart, learned when a live SET_CUR is rejected). */
static int32_t s_desired[CAM_UVC_CTRL_COUNT];
static bool s_has_desired[CAM_UVC_CTRL_COUNT];
static bool s_needs_restart[CAM_UVC_CTRL_COUNT];

static uint32_t bitmap(const uint8_t *p, uint8_t size)
{
    uint32_t v = 0;
    for (int i = 0; i < size && i < 4; i++) {
        v |= (uint32_t)p[i] << (8 * i);
    }
    return v;
}

/* Find the first Video Control interface, its Processing Unit and camera terminal. */
static void parse_config(const usb_config_desc_t *cfg, vc_layout_t *out)
{
    const uint8_t *p = (const uint8_t *)cfg;
    const uint16_t total = cfg->wTotalLength;
    bool in_vc = false, done_vc = false;
    *out = (vc_layout_t){0};

    for (uint16_t off = 0; off + 2 <= total;) {
        const uint8_t *d = p + off;
        uint8_t len = d[0];
        if (len < 2 || off + len > total) {
            break;
        }
        if (d[1] == DESC_TYPE_INTERFACE && len >= 9) {
            if (in_vc) {
                done_vc = true;     /* left the first VC interface */
            }
            in_vc = !done_vc && d[5] == USB_CLASS_VIDEO_ && d[6] == UVC_SC_VIDEOCONTROL;
            if (in_vc) {
                out->vc_intf = d[2];
            }
        } else if (in_vc && d[1] == DESC_TYPE_CS_INTERFACE && len >= 3) {
            if (d[2] == VC_HEADER && len >= 5) {
                out->bcd_uvc = d[3] | (d[4] << 8);
            } else if (d[2] == VC_PROCESSING_UNIT && len >= 8 && !out->found) {
                uint8_t size = d[7];
                if (8 + size <= len) {
                    out->found = true;
                    out->unit_id = d[3];
                    out->pu_controls = bitmap(d + 8, size);
                }
            } else if (d[2] == VC_INPUT_TERMINAL && len >= 15 && (d[4] | (d[5] << 8)) == ITT_CAMERA) {
                uint8_t size = d[14];
                if (15 + size <= len) {
                    out->ct_controls = bitmap(d + 15, size);
                }
            }
        }
        off += len;
    }
}

static void pu_client_event(const usb_host_client_event_msg_t *msg, void *arg)
{
    /* Never called: the temporary client does not handle events. */
}

/* Read the active configuration descriptor through a short-lived USB host
 * client, the same way usb_host_uvc's uvc_host_get_frame_list() does. */
static esp_err_t read_layout(uint8_t dev_addr, vc_layout_t *out)
{
    const usb_host_client_config_t cfg = {
        .is_synchronous = false,
        .max_num_event_msg = 4,
        .async = { .client_event_callback = pu_client_event, .callback_arg = NULL },
    };
    usb_host_client_handle_t client;
    ESP_RETURN_ON_ERROR(usb_host_client_register(&cfg, &client), TAG, "client register failed");

    usb_device_handle_t dev;
    esp_err_t ret = usb_host_device_open(client, dev_addr, &dev);
    if (ret == ESP_OK) {
        const usb_config_desc_t *config_desc = NULL;
        ret = usb_host_get_active_config_descriptor(dev, &config_desc);
        if (ret == ESP_OK) {
            parse_config(config_desc, out);
        }
        usb_host_device_close(client, dev);
    }
    usb_host_client_deregister(client);
    return ret;
}

/* One PU request (caller holds s_lock). The ELP camera sporadically STALLs or
 * errors a valid request (a different control on every connect), so retry. */
static esp_err_t pu_xfer(uint8_t type, uint8_t request, const pu_desc_t *pu, uint8_t len, uint8_t *buf)
{
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < PU_ATTEMPTS && err != ESP_OK; attempt++) {
        if (attempt) {
            vTaskDelay(pdMS_TO_TICKS(PU_RETRY_DELAY_MS));
        }
        err = uvc_host_usb_ctrl(s_stream, type, request, (uint16_t)pu->selector << 8,
                                ((uint16_t)s_layout.unit_id << 8) | s_layout.vc_intf, len, buf);
    }
    return err;
}

static esp_err_t pu_get(uint8_t request, const pu_desc_t *pu, uint8_t len, int32_t *value)
{
    uint8_t buf[2] = {0};
    esp_err_t err = pu_xfer(REQ_TYPE_GET, request, pu, len, buf);
    if (err != ESP_OK) {
        return err;
    }
    if (len == 1) {
        *value = buf[0];
    } else {
        uint16_t raw = buf[0] | (buf[1] << 8);
        *value = pu->is_signed ? (int16_t)raw : raw;
    }
    return ESP_OK;
}

static esp_err_t pu_set(const pu_desc_t *pu, int32_t value)
{
    uint8_t buf[2] = { (uint8_t)value, (uint8_t)(value >> 8) };
    return pu_xfer(REQ_TYPE_SET, REQ_SET_CUR, pu, pu->len, buf);
}

static void clear_locked(void)
{
    s_stream = NULL;
    memset(&s_layout, 0, sizeof(s_layout));
    memset(s_info, 0, sizeof(s_info));
}

static void notify(void)
{
    cam_uvc_ctrl_notify_fn fn = s_notify;
    if (fn) {
        fn();
    }
}

esp_err_t cam_uvc_ctrl_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    return s_lock ? ESP_OK : ESP_ERR_NO_MEM;
}

void cam_uvc_ctrl_set_notify(cam_uvc_ctrl_notify_fn fn)
{
    s_notify = fn;
}

/* True if the control is overridden by its auto partner (by what will be applied). */
static bool auto_overrides(const pu_desc_t *pu)
{
    if (pu->auto_ctrl < 0 || !s_info[pu->auto_ctrl].supported) {
        return false;
    }
    return s_has_desired[pu->auto_ctrl] ? s_desired[pu->auto_ctrl] != 0 : s_info[pu->auto_ctrl].current != 0;
}

/* Before streaming: bring the camera to the requested values. Auto controls go
 * first, since they decide whether their manual partner can be set at all. */
static void apply_desired_locked(void)
{
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < CAM_UVC_CTRL_COUNT; i++) {
            const pu_desc_t *pu = &s_pu[i];
            cam_uvc_ctrl_info_t *info = &s_info[i];
            if (pu->is_bool != (pass == 0) || !s_has_desired[i] || !info->supported ||
                    s_desired[i] == info->current || auto_overrides(pu)) {
                continue;
            }
            int32_t value = s_desired[i];
            esp_err_t err = pu_set(pu, value);
            int32_t actual = value;
            if (err == ESP_OK && pu_get(REQ_GET_CUR, pu, pu->len, &actual) == ESP_OK) {
                info->current = actual;
            }
            ESP_LOGI(TAG, "applied %s=%" PRId32 " before streaming: %s (camera reports %" PRId32 ")",
                     pu->name, value, esp_err_to_name(err), actual);
        }
    }
}

static esp_err_t probe_locked(uint8_t dev_addr, uvc_host_stream_hdl_t stream)
{
    vc_layout_t layout;
    ESP_RETURN_ON_ERROR(read_layout(dev_addr, &layout), TAG, "configuration descriptor unavailable");
    ESP_LOGI(TAG, "UVC %x.%02x, VC interface %u, camera terminal controls 0x%06" PRIx32,
             layout.bcd_uvc >> 8, layout.bcd_uvc & 0xff, layout.vc_intf, layout.ct_controls);
    ESP_RETURN_ON_FALSE(layout.found, ESP_ERR_NOT_FOUND, TAG, "no Processing Unit");
    ESP_LOGI(TAG, "Processing Unit id %u, controls 0x%06" PRIx32, layout.unit_id, layout.pu_controls);
    s_layout = layout;
    s_stream = stream;

    int supported = 0;
    for (int i = 0; i < CAM_UVC_CTRL_COUNT; i++) {
        const pu_desc_t *pu = &s_pu[i];
        if (!(layout.pu_controls & (1UL << pu->bit))) {
            continue;
        }
        cam_uvc_ctrl_info_t info = {0};
        int32_t caps;
        if (pu_get(REQ_GET_INFO, pu, 1, &caps) != ESP_OK) {
            ESP_LOGW(TAG, "  %-13s GET_INFO failed", pu->name);
            continue;
        }
        info.caps = (uint8_t)caps;
        /* MIN/MAX/RES/DEF/CUR. A failure leaves the field at 0 and is reported. */
        const struct { uint8_t req; int32_t *dst; const char *name; } reads[] = {
            { REQ_GET_MIN, &info.minimum, "MIN" }, { REQ_GET_MAX, &info.maximum, "MAX" },
            { REQ_GET_RES, &info.step, "RES" }, { REQ_GET_DEF, &info.default_value, "DEF" },
            { REQ_GET_CUR, &info.current, "CUR" },
        };
        bool all_ok = true;
        size_t first = 0;
        if (pu->is_bool) {
            info.minimum = 0;
            info.maximum = 1;
            info.step = 1;
            first = 3;      /* DEF and CUR only */
        }
        for (size_t r = first; r < sizeof(reads) / sizeof(reads[0]); r++) {
            if (pu_get(reads[r].req, pu, pu->len, reads[r].dst) != ESP_OK) {
                ESP_LOGW(TAG, "  %-13s GET_%s failed", pu->name, reads[r].name);
                all_ok = false;
            }
        }
        info.supported = all_ok;
        s_info[i] = info;
        supported += all_ok;
        ESP_LOGI(TAG, "  %-13s info=0x%02x min=%" PRId32 " max=%" PRId32 " res=%" PRId32
                      " def=%" PRId32 " cur=%" PRId32 "%s",
                 pu->name, info.caps, info.minimum, info.maximum, info.step,
                 info.default_value, info.current, all_ok ? "" : " (incomplete)");
    }
    ESP_LOGI(TAG, "%d controls usable", supported);
    apply_desired_locked();
    return ESP_OK;
}

esp_err_t cam_uvc_ctrl_probe(uint8_t dev_addr, uvc_host_stream_hdl_t stream)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    clear_locked();
    esp_err_t err = probe_locked(dev_addr, stream);
    if (err != ESP_OK) {
        clear_locked();
    }
    xSemaphoreGive(s_lock);
    notify();
    return err;
}

void cam_uvc_ctrl_forget(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool had = s_stream != NULL;
    clear_locked();
    xSemaphoreGive(s_lock);
    if (had) {
        notify();
    }
}

bool cam_uvc_ctrl_get_info(cam_uvc_ctrl_t control, cam_uvc_ctrl_info_t *out)
{
    if (!s_lock || control < 0 || control >= CAM_UVC_CTRL_COUNT) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool ok = s_info[control].supported;
    if (ok) {
        *out = s_info[control];
    }
    xSemaphoreGive(s_lock);
    return ok;
}

esp_err_t cam_uvc_ctrl_read(cam_uvc_ctrl_t control, int32_t *value)
{
    ESP_RETURN_ON_FALSE(control >= 0 && control < CAM_UVC_CTRL_COUNT && value, ESP_ERR_INVALID_ARG,
                        TAG, "bad control");
    ESP_RETURN_ON_FALSE(s_lock, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    const pu_desc_t *pu = &s_pu[control];
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = ESP_ERR_INVALID_STATE;
    if (s_stream && s_info[control].supported) {
        err = pu_get(REQ_GET_CUR, pu, pu->len, value);
        if (err == ESP_OK) {
            s_info[control].current = *value;
        }
    }
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t cam_uvc_ctrl_set(cam_uvc_ctrl_t control, int32_t value)
{
    ESP_RETURN_ON_FALSE(control >= 0 && control < CAM_UVC_CTRL_COUNT, ESP_ERR_INVALID_ARG, TAG, "bad control");
    ESP_RETURN_ON_FALSE(s_lock, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    const pu_desc_t *pu = &s_pu[control];
    xSemaphoreTake(s_lock, portMAX_DELAY);
    cam_uvc_ctrl_info_t *info = &s_info[control];
    esp_err_t err;
    bool restart = false;
    if (!s_stream || !info->supported) {
        err = ESP_ERR_INVALID_STATE;
    } else if (value < info->minimum || value > info->maximum) {
        err = ESP_ERR_INVALID_ARG;
    } else {
        s_desired[control] = value;
        s_has_desired[control] = true;
        err = ESP_FAIL;
        if (!s_needs_restart[control] && !auto_overrides(pu)) {
            err = pu_set(pu, value);
            if (err != ESP_OK) {
                s_needs_restart[control] = true;
                ESP_LOGW(TAG, "%s: rejected while streaming (%s); will apply on a camera restart",
                         pu->name, esp_err_to_name(err));
            }
        }
        if (err == ESP_OK) {
            /* Read back: the camera may clamp, round or ignore the request. */
            int32_t actual;
            if (pu_get(REQ_GET_CUR, pu, pu->len, &actual) == ESP_OK) {
                info->current = actual;
                if (actual != value) {
                    ESP_LOGW(TAG, "%s: set %" PRId32 ", camera reports %" PRId32, pu->name, value, actual);
                }
            } else {
                info->current = value;
            }
        } else {
            info->current = value;      /* shown as requested until the restart confirms it */
            restart = true;
            err = ESP_ERR_NOT_FINISHED;
        }
    }
    xSemaphoreGive(s_lock);
    if (restart) {
        cam_uvc_request_power_cycle();
    } else if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s: set %" PRId32 " failed (%s)", pu->name, value, esp_err_to_name(err));
    }
    return err;
}
