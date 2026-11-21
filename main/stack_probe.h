#pragma once

/* Diagnostic: switch the M5-Bus 5 V on, scan the I2C bus and read (only read)
 * the FAN (0x18), PPS (0x35) and LAN (W5500 on SPI) Stack modules. */
void stack_probe_run(void);
