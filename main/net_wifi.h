#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Wi-Fi through the on-board ESP32-C6 (esp_hosted over SDIO): an access point
 * that is always up while Wi-Fi is on, plus a station when a network is saved.
 * The web page (web_server.c) runs while Wi-Fi is on. */

typedef enum {
    NET_OFF,
    NET_STARTING,       /* powering and connecting the C6, or starting Wi-Fi */
    NET_ON,
    NET_FAILED,         /* the C6 did not answer or Wi-Fi failed to start */
} net_state_t;

typedef struct {
    net_state_t state;
    bool sta_configured;
    bool sta_connected;
    char sta_ssid[33];
    uint32_t sta_ip;        /* lwIP byte order (esp_ip4_addr_t.addr) */
    int8_t sta_rssi;
    uint8_t channel;
    uint8_t ap_clients;
    uint32_t ap_ip;
    char c6_version[16];    /* "1.4.1", empty until the C6 has answered */
} net_status_t;

/* Opens NVS, loads the saved settings and starts the Wi-Fi task. Wi-Fi stays off. */
esp_err_t net_wifi_init(void);

/* Asynchronous: the Wi-Fi task powers up the C6 on the first start. */
void net_wifi_enable(bool on);

void net_wifi_get_status(net_status_t *out);

/* Settings page. NULL keeps a value. ap_pass must be 8..63 characters;
 * sta_ssid may be empty (no station); sta_pass may be empty (open network) or 8..63.
 * Saved to NVS and applied (Wi-Fi restarts after APP_WIFI_APPLY_DELAY_MS). */
esp_err_t net_wifi_save_settings(const char *ap_pass, const char *sta_ssid, const char *sta_pass);
