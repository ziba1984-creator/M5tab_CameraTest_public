#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "frame_mbox.h"

/* Logical landscape layout (1280x720):
 *   y 0..40     page tabs (always)
 *   camera page: y 40..640 the two camera panes, y 640..720 the control bar
 *   other pages: y 40..720 the page image */
#define CAMERA_UI_TAB_HEIGHT  40
#define CAMERA_UI_Y          640
#define CAMERA_UI_HEIGHT      80

typedef enum {
    CAMERA_UI_PAGE_CAMERA,
    CAMERA_UI_PAGE_MEASURE,     /* placeholder for docked measurement modules */
    CAMERA_UI_PAGE_SETTINGS,
    CAMERA_UI_PAGE_COUNT,
} camera_ui_page_t;

/* Images the display draws; NULL when not shown on the current page. All are
 * RGB565 at half the screen resolution (the display scales them by 2). */
typedef struct {
    const video_frame_t *tabs;  /* -> y 0..CAMERA_UI_TAB_HEIGHT */
    const video_frame_t *bar;   /* camera page: -> y CAMERA_UI_Y..720 */
    const video_frame_t *page;  /* other pages: -> y CAMERA_UI_TAB_HEIGHT..720 */
} camera_ui_frames_t;

/* Read the initial control values, render the bar and start the UI task. */
esp_err_t camera_ui_init(void);

/* Touch-task callback: only posts the tapped item to the UI task. */
void camera_ui_on_touch(uint16_t x, uint16_t y);

/* Page currently shown (any task). */
camera_ui_page_t camera_ui_page(void);

/* Display task: lock the current images for drawing, then release them.
 * Returns false (and nothing needs releasing) if the UI is disabled. */
bool camera_ui_acquire(camera_ui_frames_t *out);
void camera_ui_release(void);
