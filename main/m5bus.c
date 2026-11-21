#include "esp_check.h"
#include "esp_io_expander.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp/m5stack_tab5.h"
#include "m5bus.h"

static const char *TAG = "m5bus";

#define EXT5V_EN_PIN    IO_EXPANDER_PIN_NUM_2   /* PI4IOE5V6408 at 0x43 (BSP_IO_EXPANDER_ADDRESS), P2 */
#define MODULE_BOOT_MS  300                     /* module MCUs answer on I2C after this */

esp_err_t m5bus_power_on(void)
{
    static bool on;
    if (on) {
        return ESP_OK;
    }
    esp_io_expander_handle_t io = bsp_io_expander_init();
    ESP_RETURN_ON_FALSE(io, ESP_FAIL, TAG, "IO expander unavailable");
    esp_err_t err = esp_io_expander_set_dir(io, EXT5V_EN_PIN, IO_EXPANDER_OUTPUT);
    if (err == ESP_OK) err = esp_io_expander_set_output_mode(io, EXT5V_EN_PIN, IO_EXPANDER_OUTPUT_MODE_PUSH_PULL);
    if (err == ESP_OK) err = esp_io_expander_set_level(io, EXT5V_EN_PIN, 1);
    ESP_RETURN_ON_ERROR(err, TAG, "EXT5V_EN");
    vTaskDelay(pdMS_TO_TICKS(MODULE_BOOT_MS));
    on = true;
    ESP_LOGI(TAG, "M5-Bus 5 V on");
    return ESP_OK;
}
