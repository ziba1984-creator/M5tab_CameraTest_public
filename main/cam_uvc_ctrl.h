#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "usb/uvc_host.h"

/* Image controls of the external UVC camera's Processing Unit. All functions
 * are thread-safe; transfers are serialized and never outlive the stream. */

typedef enum {
    CAM_UVC_CTRL_BRIGHTNESS,
    CAM_UVC_CTRL_CONTRAST,
    CAM_UVC_CTRL_HUE,
    CAM_UVC_CTRL_SATURATION,
    CAM_UVC_CTRL_SHARPNESS,
    CAM_UVC_CTRL_GAMMA,
    CAM_UVC_CTRL_WB_TEMP,
    CAM_UVC_CTRL_BACKLIGHT,
    CAM_UVC_CTRL_GAIN,
    CAM_UVC_CTRL_POWER_LINE,
    CAM_UVC_CTRL_HUE_AUTO,
    CAM_UVC_CTRL_WB_TEMP_AUTO,
    CAM_UVC_CTRL_CONTRAST_AUTO,
    CAM_UVC_CTRL_COUNT,
} cam_uvc_ctrl_t;

typedef struct {
    bool supported;         /* advertised by the descriptor and GET_INFO answered */
    uint8_t caps;           /* GET_INFO: D0 get, D1 set, D2 disabled by auto mode, D3 auto-update */
    int32_t minimum;
    int32_t maximum;
    int32_t step;
    int32_t default_value;
    int32_t current;
} cam_uvc_ctrl_info_t;

/* Called (from the UVC manager task) after a probe or forget changed what is known. */
typedef void (*cam_uvc_ctrl_notify_fn)(void);

esp_err_t cam_uvc_ctrl_init(void);
void cam_uvc_ctrl_set_notify(cam_uvc_ctrl_notify_fn fn);

/* Read the Processing Unit from the camera's configuration descriptor, query
 * GET_INFO/MIN/MAX/RES/DEF/CUR of every advertised control and apply the
 * requested values. Call once per connection, before the stream is started. */
esp_err_t cam_uvc_ctrl_probe(uint8_t dev_addr, uvc_host_stream_hdl_t stream);

/* Drop the probed state. Must be called before the stream is closed. */
void cam_uvc_ctrl_forget(void);

/* Probed info of one control (cached, no USB traffic); false if unknown or unsupported. */
bool cam_uvc_ctrl_get_info(cam_uvc_ctrl_t control, cam_uvc_ctrl_info_t *out);

/* GET_CUR from the camera, updating the cache. */
esp_err_t cam_uvc_ctrl_read(cam_uvc_ctrl_t control, int32_t *value);

/* Request a value. It is remembered and re-applied on every (re)connect.
 * ESP_OK: applied live (the cache holds what the camera reports, it may clamp).
 * ESP_ERR_NOT_FINISHED: the camera rejects this control while streaming, so the
 * camera is power-cycled (debounced) and the value applied before streaming. */
esp_err_t cam_uvc_ctrl_set(cam_uvc_ctrl_t control, int32_t value);
