#pragma once

#include "esp_err.h"

/* Web page on port 80 while Wi-Fi is on (started and stopped by net_wifi):
 *   GET  /                 status, recordings and Wi-Fi settings (one page)
 *   GET  /api/status       JSON counters; the page turns them into rates
 *   GET  /api/files        JSON list of the recordings and the card's free space
 *   GET  /rec/Rnnnn.AVI    download one recording (one download at a time)
 *   POST /api/settings     form fields ap_pass, sta_ssid, sta_pass */
esp_err_t web_server_start(void);
void web_server_stop(void);
