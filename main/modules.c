#include <string.h>
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "bsp/m5stack_tab5.h"
#include "app_config.h"
#include "m5bus.h"
#include "modules.h"
#include "net_eth.h"

static const char *TAG = "modules";

#define FAN_ADDR            0x18
#define PPS_ADDR            0x35
#define I2C_HZ              100000
#define I2C_TIMEOUT_MS      50

/* Module Fan v1.1 registers (M5Module-Fan library) */
#define FAN_REG_ENABLE      0x00
#define FAN_REG_PWM_FREQ    0x10
#define FAN_REG_DUTY        0x20
#define FAN_REG_RPM         0x30    /* uint16, little endian */
#define FAN_REG_FW          0xFE

static SemaphoreHandle_t s_lock;    /* the fields below and the device handles */
static i2c_master_dev_handle_t s_fan_dev;
static fan_state_t s_fan;
/* The FAN sometimes answers a poll with 0 in its config registers (seen: enable
 * and PWM frequency both 0 for one read while the fan kept turning at ~4900 RPM).
 * A config change is shown only once two reads in a row agree. */
static struct { uint8_t en, freq, duty; bool valid; } s_fan_last;
static bool s_pps_present;

static const char *const s_names[MODULE_COUNT] = {
    [MODULE_LAN] = "LAN 13.2",
    [MODULE_FAN] = "FAN",
    [MODULE_PPS] = "PPS 13.2",
};

const char *module_name(module_id_t id)
{
    return id < MODULE_COUNT ? s_names[id] : "?";
}

bool module_present(module_id_t id)
{
    if (id == MODULE_LAN) {
        eth_status_t eth;
        net_eth_get_status(&eth);
        return eth.present;
    }
    if (!s_lock) {
        return false;       /* modules_init() not run (or failed) */
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool p = id == MODULE_FAN ? s_fan.present : id == MODULE_PPS ? s_pps_present : false;
    xSemaphoreGive(s_lock);
    return p;
}

/* ---- FAN ----------------------------------------------------------------------- */

static esp_err_t fan_read(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_fan_dev, &reg, 1, buf, len, I2C_TIMEOUT_MS);
}

static esp_err_t fan_write(uint8_t reg, uint8_t value)
{
    const uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(s_fan_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

/* Caller holds s_lock. */
/* why: who asked for the read (for the log). */
static void fan_poll_locked(const char *why)
{
    if (!s_fan_dev) {
        const i2c_device_config_t cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = FAN_ADDR, .scl_speed_hz = I2C_HZ,
        };
        if (i2c_master_bus_add_device(bsp_i2c_get_handle(), &cfg, &s_fan_dev) != ESP_OK) {
            return;
        }
        if (fan_read(FAN_REG_FW, &s_fan.fw, 1) != ESP_OK) {
            i2c_master_bus_rm_device(s_fan_dev);
            s_fan_dev = NULL;
            return;
        }
        ESP_LOGI(TAG, "FAN found (firmware %u)", s_fan.fw);
    }
    uint8_t en = 0, freq = 0, duty = 0, rpm[2] = {0};
    esp_err_t err = fan_read(FAN_REG_ENABLE, &en, 1);
    if (err == ESP_OK) err = fan_read(FAN_REG_PWM_FREQ, &freq, 1);
    if (err == ESP_OK) err = fan_read(FAN_REG_DUTY, &duty, 1);
    if (err == ESP_OK) err = fan_read(FAN_REG_RPM, rpm, 2);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "FAN gone (%s)", esp_err_to_name(err));
        i2c_master_bus_rm_device(s_fan_dev);
        s_fan_dev = NULL;
        memset(&s_fan, 0, sizeof(s_fan));
        s_fan_last.valid = false;
        return;
    }
    const bool repeated = s_fan_last.valid && s_fan_last.en == en && s_fan_last.freq == freq && s_fan_last.duty == duty;
    const bool differs = (en != 0) != s_fan.enabled || freq != s_fan.pwm_freq || duty != s_fan.duty;
    if (!s_fan.present || repeated) {
        s_fan.enabled = en != 0;
        s_fan.pwm_freq = freq;
        s_fan.duty = duty;
    } else if (differs) {
        ESP_LOGI(TAG, "FAN read differs, waiting for a second read (%s: enable %u freq %u duty %u)", why, en, freq, duty);
    }
    s_fan_last.en = en;
    s_fan_last.freq = freq;
    s_fan_last.duty = duty;
    s_fan_last.valid = true;
    s_fan.present = true;
    s_fan.rpm = rpm[0] | (rpm[1] << 8);
}

void fan_get(fan_state_t *out)
{
    if (!s_lock) {
        memset(out, 0, sizeof(*out));
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_fan;
    xSemaphoreGive(s_lock);
}

static esp_err_t fan_set(uint8_t reg, uint8_t value)
{
    if (!s_lock) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = s_fan_dev ? fan_write(reg, value) : ESP_ERR_INVALID_STATE;
    if (err == ESP_OK) {
        /* Show the written value at once; a read-back right after the write can
         * hit the module's occasional zero answers. The next polls confirm it. */
        if (reg == FAN_REG_ENABLE) {
            s_fan.enabled = value != 0;
        } else if (reg == FAN_REG_DUTY) {
            s_fan.duty = value;
        }
        s_fan_last.valid = false;
        ESP_LOGI(TAG, "FAN write reg 0x%02x = %u", reg, value);
    }
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t fan_set_duty(uint8_t percent)
{
    return fan_set(FAN_REG_DUTY, percent > 100 ? 100 : percent);
}

esp_err_t fan_set_enabled(bool on)
{
    return fan_set(FAN_REG_ENABLE, on ? 1 : 0);
}

/* ---- polling -------------------------------------------------------------------- */

static void poll_task(void *arg)
{
    for (;;) {
        bool pps = i2c_master_probe(bsp_i2c_get_handle(), PPS_ADDR, I2C_TIMEOUT_MS) == ESP_OK;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        fan_poll_locked("poll");
        if (pps != s_pps_present) {
            ESP_LOGI(TAG, "PPS %s", pps ? "found" : "gone");
            s_pps_present = pps;
        }
        xSemaphoreGive(s_lock);
        vTaskDelay(pdMS_TO_TICKS(APP_MODULE_POLL_MS));
    }
}

esp_err_t modules_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_lock, ESP_ERR_NO_MEM, TAG, "no memory");
    ESP_RETURN_ON_ERROR(m5bus_power_on(), TAG, "bus power");
    ESP_RETURN_ON_FALSE(xTaskCreate(poll_task, "modules", APP_TASK_MODULES_STACK, NULL, APP_TASK_MODULES_PRIO,
                                    NULL) == pdPASS, ESP_ERR_NO_MEM, TAG, "task create failed");
    return ESP_OK;
}
