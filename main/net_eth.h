#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Wired Ethernet through the M5Stack LAN Module 13.2 (W5500 on the M5-Bus SPI).
 * Started once at boot when the module answers; the web page then serves on it
 * too. IP settings (DHCP, or a fixed address, netmask and gateway) are kept in NVS. */

typedef struct {
    bool present;           /* a W5500 answered at boot */
    bool link;              /* cable connected (link up) */
    bool dhcp;              /* configured mode */
    uint32_t ip, mask, gw;  /* current, lwIP byte order; ip 0 = none yet */
    uint32_t cfg_ip, cfg_mask, cfg_gw;  /* saved fixed-address settings */
    uint8_t mac[6];
} eth_status_t;

/* Call after nvs_flash_init(). ESP_ERR_NOT_FOUND: no LAN module (not an error for the app). */
esp_err_t net_eth_init(void);

void net_eth_get_status(eth_status_t *out);

/* Web page: validate, save to NVS and apply after APP_ETH_APPLY_DELAY_MS (so the
 * reply still reaches the browser). With dhcp, the address fields are kept for later.
 * ESP_ERR_INVALID_ARG: inconsistent fixed-address settings. */
esp_err_t net_eth_save_settings(bool dhcp, uint32_t ip, uint32_t mask, uint32_t gw);
