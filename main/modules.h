#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Stack modules on the M5-Bus. A background task looks for the I2C modules
 * every APP_MODULE_POLL_MS (they may appear later, e.g. the PPS once its DC
 * input is powered) and keeps their readings. To add a module: a new id here,
 * its name and detection in modules.c, and its page in measure_ui.c. */

typedef enum {
    MODULE_LAN,     /* LAN Module 13.2 (W5500, SPI): see net_eth */
    MODULE_FAN,     /* Module Fan v1.1 (I2C 0x18) */
    MODULE_PPS,     /* Module13.2 PPS (I2C 0x35), shown only; control comes later */
    MODULE_COUNT,
} module_id_t;

typedef struct {
    bool present;
    bool enabled;
    uint8_t duty;       /* % */
    uint8_t pwm_freq;   /* 0 = 1 kHz, 1 = 12 kHz, 2 = 24 kHz, 3 = 48 kHz */
    uint8_t fw;
    uint16_t rpm;
} fan_state_t;

/* Switches the bus 5 V on and starts the polling task. */
esp_err_t modules_init(void);

bool module_present(module_id_t id);
const char *module_name(module_id_t id);

void fan_get(fan_state_t *out);
/* UI task. Written to the module at once (not saved in its flash: the
 * module starts with its own defaults after a power cycle). */
esp_err_t fan_set_duty(uint8_t percent);
esp_err_t fan_set_enabled(bool on);
