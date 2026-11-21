#pragma once

#include "esp_err.h"

/* Web page on port 80, on every interface, while Wi-Fi is on or the LAN module is present:
 *   GET  /                 status, recordings and Wi-Fi settings (one page)
 *   GET  /api/status       JSON counters; the page turns them into rates
 *   GET  /api/files        JSON list of the recordings and the card's free space
 *   GET  /rec/Rnnnn.AVI    download one recording (one download at a time)
 *   POST /api/settings     form fields ap_pass, sta_ssid, sta_pass */
/* Who needs the page. It runs while at least one user has started it. */
typedef enum {
    WEB_USER_WIFI = 1 << 0,
    WEB_USER_ETH  = 1 << 1,
} web_user_t;

esp_err_t web_server_start(web_user_t user);
/* Drops `user`; the server stops when no user is left. Then it waits for a running
 * download to end first. ESP_ERR_TIMEOUT: it did not end within 15 s, so the server
 * was left running (it refuses new downloads) and `user` still counts as a user. */
esp_err_t web_server_stop(web_user_t user);
