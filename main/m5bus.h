#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

/* Tab5 M5-Bus (rear 30-pin connector, docs.m5stack.com/en/core/Tab5) for Stack
 * modules. Its I2C is the internal bus (G31/G32, bsp_i2c_get_handle()). */
#define M5BUS_SPI_HOST  SPI2_HOST
#define M5BUS_SCK       GPIO_NUM_5      /* bus pin 11 */
#define M5BUS_MOSI      GPIO_NUM_18     /* bus pin 7 */
#define M5BUS_MISO      GPIO_NUM_19     /* bus pin 9 */
/* LAN Module 13.2: chip select on bus pin 20 (jumper; found on G4 on this unit).
 * Its INT (pin 2 / 26) and RST (pin 22 / 24) jumpers are not used. */
#define M5BUS_LAN_CS    GPIO_NUM_4

/* Switch on the 5 V the Tab5 supplies to the M5-Bus (EXT5V_EN, IO expander 0x43 P2).
 * It is off after reset; modules powered from the bus (LAN, FAN) need it. Idempotent. */
esp_err_t m5bus_power_on(void);
