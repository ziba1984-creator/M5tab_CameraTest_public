#pragma once

#include "esp_err.h"
#include "frame_mbox.h"

typedef enum {
    CAM_BUILTIN_CTRL_BRIGHTNESS,
    CAM_BUILTIN_CTRL_CONTRAST,
    CAM_BUILTIN_CTRL_SATURATION,
    CAM_BUILTIN_CTRL_HUE,
    CAM_BUILTIN_CTRL_EXPOSURE,
    CAM_BUILTIN_CTRL_GAIN,
    CAM_BUILTIN_CTRL_COUNT,
} cam_builtin_ctrl_t;

typedef struct {
    int32_t minimum;
    int32_t maximum;
    int32_t step;
    int32_t default_value;
    int32_t current;
} cam_builtin_ctrl_info_t;

typedef struct {
    bool enable;
    float matrix[3][3];
} cam_builtin_ccm_t;

/* Built-in SC202CS camera (MIPI-CSI -> ISP -> RGB565) via esp_video / V4L2. */

/* Power up the camera, open /dev/video0 and set up the capture buffers. */
esp_err_t cam_builtin_init(void);

/* Mailbox the captured frames are published to (valid after init). */
frame_mbox_t cam_builtin_mbox(void);

/* Start streaming and the capture task. */
esp_err_t cam_builtin_start(void);

/* Query or change a live sensor/ISP control. */
esp_err_t cam_builtin_ctrl_query(cam_builtin_ctrl_t control, cam_builtin_ctrl_info_t *out);
esp_err_t cam_builtin_ctrl_set(cam_builtin_ctrl_t control, int32_t value);

/* Read or replace the ISP colour-correction matrix. Intended for capability
 * probing first; the automatic image pipeline may update this matrix later. */
esp_err_t cam_builtin_ccm_get(cam_builtin_ccm_t *out);
esp_err_t cam_builtin_ccm_set(const cam_builtin_ccm_t *ccm);
