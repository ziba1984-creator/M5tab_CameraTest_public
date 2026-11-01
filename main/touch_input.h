#pragma once

#include <stdint.h>
#include "esp_err.h"

typedef void (*touch_input_press_cb_t)(uint16_t logical_x, uint16_t logical_y);

/* Initialize the Tab5 touch controller and start coordinate monitoring. */
esp_err_t touch_input_init(void);
esp_err_t touch_input_start(void);
void touch_input_set_press_callback(touch_input_press_cb_t callback);
