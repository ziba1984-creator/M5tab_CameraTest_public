/* Diagnostic (APP_STACK_PROBE): what is on the M5-Bus. Read-only towards the
 * modules: nothing is switched on except the bus 5 V (EXT5V_EN). */
#include <string.h>
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_io_expander.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp/m5stack_tab5.h"
#include "app_config.h"
#include "stack_probe.h"

static const char *TAG = "stack_probe";

/* Tab5 M5-Bus (docs.m5stack.com/en/core/Tab5): SPI and the LAN module's jumper options. */
#define BUS_SCK         GPIO_NUM_5
#define BUS_MOSI        GPIO_NUM_18
#define BUS_MISO        GPIO_NUM_19
#define EXT5V_EN_PIN    IO_EXPANDER_PIN_NUM_2   /* PI4IOE5V6408 at 0x43, P2 */

#define FAN_ADDR        0x18
#define PPS_ADDR        0x35

static esp_err_t reg_read(i2c_master_dev_handle_t dev, uint8_t reg, void *buf, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, 100);
}

static i2c_master_dev_handle_t add_dev(uint8_t addr)
{
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev = NULL;
    i2c_master_bus_add_device(bsp_i2c_get_handle(), &cfg, &dev);
    return dev;
}

static void probe_fan(void)
{
    i2c_master_dev_handle_t dev = add_dev(FAN_ADDR);
    if (!dev) {
        return;
    }
    uint8_t ver = 0, ctrl = 0, freq = 0, duty = 0, rpm[2] = {0}, sig[2] = {0};
    esp_err_t e1 = reg_read(dev, 0xFE, &ver, 1);
    esp_err_t e2 = reg_read(dev, 0x00, &ctrl, 1);
    esp_err_t e3 = reg_read(dev, 0x10, &freq, 1);
    esp_err_t e4 = reg_read(dev, 0x20, &duty, 1);
    esp_err_t e5 = reg_read(dev, 0x30, rpm, 2);
    esp_err_t e6 = reg_read(dev, 0x40, sig, 2);
    ESP_LOGW(TAG, "FAN 0x18: fw %u, enable %u, pwm freq code %u, duty %u %%, rpm %u, signal %u (errors %s %s %s %s %s %s)",
             ver, ctrl, freq, duty, rpm[0] | (rpm[1] << 8), sig[0] | (sig[1] << 8), esp_err_to_name(e1),
             esp_err_to_name(e2), esp_err_to_name(e3), esp_err_to_name(e4), esp_err_to_name(e5), esp_err_to_name(e6));
    i2c_master_bus_rm_device(dev);
}

static void probe_pps(void)
{
    i2c_master_dev_handle_t dev = add_dev(PPS_ADDR);
    if (!dev) {
        return;
    }
    uint8_t id[2] = {0}, hw = 0, sw = 0, en = 0, mode = 0;
    float vout = 0, iout = 0, temp = 0, vin = 0, vset = 0, iset = 0;
    esp_err_t e = reg_read(dev, 0x00, id, 2);
    reg_read(dev, 0x02, &hw, 1);
    reg_read(dev, 0x03, &sw, 1);
    reg_read(dev, 0x04, &en, 1);
    reg_read(dev, 0x05, &mode, 1);
    reg_read(dev, 0x08, &vout, 4);
    reg_read(dev, 0x0C, &iout, 4);
    reg_read(dev, 0x10, &temp, 4);
    reg_read(dev, 0x14, &vin, 4);
    reg_read(dev, 0x18, &vset, 4);
    reg_read(dev, 0x1C, &iset, 4);
    ESP_LOGW(TAG, "PPS 0x35: id 0x%04x hw %u sw %u | enable %u mode %u | vin %.2f V | set %.3f V %.3f A | "
             "out %.3f V %.3f A | temp %.1f (first read %s)",
             id[0] | (id[1] << 8), hw, sw, en, mode, vin, vset, iset, vout, iout, temp, esp_err_to_name(e));
    i2c_master_bus_rm_device(dev);
}

/* W5500 VERSIONR (0x0039, common block, read, variable length): 0x04 on a W5500. */
static void probe_w5500(void)
{
    const spi_bus_config_t bus = {
        .mosi_io_num = BUS_MOSI,
        .miso_io_num = BUS_MISO,
        .sclk_io_num = BUS_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed (%s)", esp_err_to_name(err));
        return;
    }
    /* Leave the reset candidates released (the module's RSTn is active low). */
    const gpio_num_t rst_pins[] = { GPIO_NUM_48, GPIO_NUM_35 };
    for (int i = 0; i < 2; i++) {
        gpio_reset_pin(rst_pins[i]);
        gpio_set_direction(rst_pins[i], GPIO_MODE_INPUT);
        gpio_pullup_en(rst_pins[i]);
    }
    const gpio_num_t int_pins[] = { GPIO_NUM_16, GPIO_NUM_51 };
    for (int i = 0; i < 2; i++) {
        gpio_reset_pin(int_pins[i]);
        gpio_set_direction(int_pins[i], GPIO_MODE_INPUT);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    const gpio_num_t cs_pins[] = { GPIO_NUM_4, GPIO_NUM_47 };
    for (int i = 0; i < 2; i++) {
        const spi_device_interface_config_t dev_cfg = {
            .clock_speed_hz = 1000000,
            .mode = 0,
            .spics_io_num = cs_pins[i],
            .queue_size = 1,
        };
        spi_device_handle_t dev;
        if (spi_bus_add_device(SPI2_HOST, &dev_cfg, &dev) != ESP_OK) {
            continue;
        }
        uint8_t tx[4] = { 0x00, 0x39, 0x00, 0x00 }, rx[4] = {0};
        spi_transaction_t t = { .length = 32, .tx_buffer = tx, .rx_buffer = rx };
        err = spi_device_polling_transmit(dev, &t);
        ESP_LOGW(TAG, "W5500 with CS on G%d: VERSIONR 0x%02x (%s)%s", cs_pins[i], rx[3], esp_err_to_name(err),
                 rx[3] == 0x04 ? "  <-- W5500 found" : "");
        spi_bus_remove_device(dev);
    }
    for (int i = 0; i < 2; i++) {
        ESP_LOGW(TAG, "INT candidate G%d level %d, RST candidate G%d level %d", int_pins[i],
                 gpio_get_level(int_pins[i]), rst_pins[i], gpio_get_level(rst_pins[i]));
    }
    spi_bus_free(SPI2_HOST);
}

void stack_probe_run(void)
{
    esp_io_expander_handle_t io = bsp_io_expander_init();
    if (io) {
        uint32_t level = 0;
        esp_io_expander_get_level(io, EXT5V_EN_PIN, &level);
        ESP_LOGW(TAG, "EXT5V_EN (0x43 P2) was %s; switching the bus 5 V on", level ? "high" : "low");
        esp_io_expander_set_dir(io, EXT5V_EN_PIN, IO_EXPANDER_OUTPUT);
        esp_io_expander_set_output_mode(io, EXT5V_EN_PIN, IO_EXPANDER_OUTPUT_MODE_PUSH_PULL);
        esp_io_expander_set_level(io, EXT5V_EN_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(300));     /* let module MCUs boot */
    }
    char found[200] = "";
    size_t n = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (i2c_master_probe(bsp_i2c_get_handle(), a, 50) == ESP_OK && n < sizeof(found) - 6) {
            n += snprintf(found + n, sizeof(found) - n, " 0x%02x", a);
        }
    }
    ESP_LOGW(TAG, "I2C (G31/G32) devices:%s", found);
    probe_fan();
    probe_pps();
    probe_w5500();
}
