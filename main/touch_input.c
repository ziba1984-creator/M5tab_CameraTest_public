#include "esp_check.h"
#include "esp_lcd_touch.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp/touch.h"
#include "app_config.h"
#include "touch_input.h"

static const char *TAG = "touch";
static esp_lcd_touch_handle_t s_touch;
static touch_input_press_cb_t s_press_callback;

static void physical_to_logical(uint16_t px, uint16_t py, uint16_t *lx, uint16_t *ly)
{
#if APP_DISPLAY_ROT_DEG == 90
    *lx = py < APP_LCD_PHYS_H ? APP_LCD_PHYS_H - 1 - py : 0;
    *ly = px < APP_LCD_PHYS_W ? px : APP_LCD_PHYS_W - 1;
#elif APP_DISPLAY_ROT_DEG == 270
    *lx = py < APP_LCD_PHYS_H ? py : APP_LCD_PHYS_H - 1;
    *ly = px < APP_LCD_PHYS_W ? APP_LCD_PHYS_W - 1 - px : 0;
#else
#error "APP_DISPLAY_ROT_DEG must be 90 or 270"
#endif
}

static void touch_task(void *arg)
{
    bool was_pressed = false;
    TickType_t last_log = 0;

    for (;;) {
        esp_err_t err = esp_lcd_touch_read_data(s_touch);
        if (err == ESP_OK) {
            esp_lcd_touch_point_data_t point;
            uint8_t count = 0;
            bool pressed = esp_lcd_touch_get_data(s_touch, &point, &count, 1) == ESP_OK && count > 0;

            if (pressed) {
                TickType_t now = xTaskGetTickCount();
                uint16_t lx, ly;
                physical_to_logical(point.x, point.y, &lx, &ly);
                if (!was_pressed && s_press_callback) {
                    s_press_callback(lx, ly);
                }
                if (!was_pressed || now - last_log >= pdMS_TO_TICKS(APP_TOUCH_LOG_PERIOD_MS)) {
                    ESP_LOGI(TAG, "press phys=(%u,%u) logical=(%u,%u)",
                             point.x, point.y, lx, ly);
                    last_log = now;
                }
            } else if (was_pressed) {
                ESP_LOGI(TAG, "release");
            }
            was_pressed = pressed;
        } else {
            ESP_LOGW(TAG, "read failed (%s)", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(APP_TOUCH_POLL_MS));
    }
}

void touch_input_set_press_callback(touch_input_press_cb_t callback)
{
    s_press_callback = callback;
}

esp_err_t touch_input_init(void)
{
    ESP_RETURN_ON_ERROR(bsp_touch_new(NULL, &s_touch), TAG, "touch init failed");
    ESP_LOGI(TAG, "ready; physical %ux%u -> logical %ux%u, rotation=%d",
             APP_LCD_PHYS_W, APP_LCD_PHYS_H, APP_LCD_PHYS_H, APP_LCD_PHYS_W,
             APP_DISPLAY_ROT_DEG);
    return ESP_OK;
}

esp_err_t touch_input_start(void)
{
    ESP_RETURN_ON_FALSE(s_touch, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(xTaskCreate(touch_task, "touch", APP_TASK_TOUCH_STACK, NULL,
                                    APP_TASK_TOUCH_PRIO, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task create failed");
    return ESP_OK;
}
