#pragma once

#include <stdbool.h>
#include "ui_draw.h"

/* MEASURE page: buttons for the connected Stack modules, and each module's own
 * screen. All calls come from the UI task. Coordinates are in the page image
 * (half the screen resolution, as drawn into `c`). */

void measure_ui_render(canvas_t *c);

/* A tap at (x, y) of the page image. True: the page must be redrawn. */
bool measure_ui_tap(int x, int y);

/* Called periodically. True: what the page shows has changed (redraw). */
bool measure_ui_poll(void);
