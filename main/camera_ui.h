#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "frame_mbox.h"

#define CAMERA_UI_Y       640
#define CAMERA_UI_HEIGHT   80

/* Read the initial control values, render the bar and start the UI task. */
esp_err_t camera_ui_init(void);

/* Touch-task callback: only posts the tapped button to the UI task. */
void camera_ui_on_touch(uint16_t x, uint16_t y);

/* Display task: lock the current bar image for drawing, then release it.
 * acquire returns NULL (and nothing needs releasing) if the UI is disabled. */
const video_frame_t *camera_ui_acquire_frame(void);
void camera_ui_release_frame(void);
