#include <inttypes.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include "esp_check.h"
#include "esp_event.h"
#include "esp_hosted.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mdns.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "bsp/m5stack_tab5.h"
#include "app_config.h"
#include "net_wifi.h"
#include "web_server.h"

static const char *TAG = "net_wifi";

#define NVS_NS  "net"

typedef enum { CMD_ON, CMD_OFF, CMD_APPLY } net_cmd_t;

typedef struct {
    char ap_pass[65];
    char sta_ssid[33];
    char sta_pass[65];
} net_settings_t;

static QueueHandle_t s_cmd;
static SemaphoreHandle_t s_lock;        /* s_set and s_st */
static net_settings_t s_set;
static net_status_t s_st;

/* Wi-Fi task only */
static bool s_c6_ok;                    /* the C6 answered over SDIO */
static bool s_stack_ok;                 /* netif, event loop, Wi-Fi driver, mDNS set up */
static esp_netif_t *s_sta_netif, *s_ap_netif;

static atomic_bool s_running;           /* between esp_wifi_start and esp_wifi_stop */
static atomic_llong s_retry_at;         /* next station connect attempt (us), 0 = none */
static atomic_bool s_apply_pending;     /* saved settings not yet applied to running Wi-Fi */

static void set_state(net_state_t state)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_st.state = state;
    xSemaphoreGive(s_lock);
}

/* ---- settings ------------------------------------------------------------------ */

static void nvs_read(nvs_handle_t h, const char *key, char *dst, size_t size)
{
    char buf[65];
    size_t n = sizeof(buf);
    if (nvs_get_str(h, key, buf, &n) == ESP_OK) {
        strlcpy(dst, buf, size);
    }
}

static void load_settings(void)
{
    strlcpy(s_set.ap_pass, APP_WIFI_AP_PASS, sizeof(s_set.ap_pass));
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_read(h, "ap_pass", s_set.ap_pass, sizeof(s_set.ap_pass));
        nvs_read(h, "sta_ssid", s_set.sta_ssid, sizeof(s_set.sta_ssid));
        nvs_read(h, "sta_pass", s_set.sta_pass, sizeof(s_set.sta_pass));
        nvs_close(h);
    }
    s_st.sta_configured = s_set.sta_ssid[0] != '\0';
    strlcpy(s_st.sta_ssid, s_set.sta_ssid, sizeof(s_st.sta_ssid));
}

static bool printable(const char *s, size_t min, size_t max)
{
    size_t n = strlen(s);
    if (n < min || n > max) {
        return false;
    }
    for (; *s; s++) {
        if ((unsigned char)*s < 0x20 || (unsigned char)*s > 0x7e) {
            return false;
        }
    }
    return true;
}

esp_err_t net_wifi_save_settings(const char *ap_pass, const char *sta_ssid, const char *sta_pass)
{
    if ((ap_pass && !printable(ap_pass, 8, 63)) ||
        (sta_ssid && strlen(sta_ssid) > 32) ||
        (sta_pass && sta_pass[0] && !printable(sta_pass, 8, 63))) {
        return ESP_ERR_INVALID_ARG;
    }
    if (sta_ssid) {
        for (const char *p = sta_ssid; *p; p++) {
            if ((unsigned char)*p < 0x20) {
                return ESP_ERR_INVALID_ARG;
            }
        }
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    net_settings_t next = s_set;
    if (ap_pass) strlcpy(next.ap_pass, ap_pass, sizeof(next.ap_pass));
    if (sta_ssid) strlcpy(next.sta_ssid, sta_ssid, sizeof(next.sta_ssid));
    if (sta_pass) strlcpy(next.sta_pass, sta_pass, sizeof(next.sta_pass));
    if (!next.sta_ssid[0]) {
        next.sta_pass[0] = '\0';
    }
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_str(h, "ap_pass", next.ap_pass);
        if (err == ESP_OK) err = nvs_set_str(h, "sta_ssid", next.sta_ssid);
        if (err == ESP_OK) err = nvs_set_str(h, "sta_pass", next.sta_pass);
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
    }
    if (err == ESP_OK) {
        s_set = next;
        s_st.sta_configured = next.sta_ssid[0] != '\0';
        strlcpy(s_st.sta_ssid, next.sta_ssid, sizeof(s_st.sta_ssid));
    }
    xSemaphoreGive(s_lock);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "settings not saved: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "settings saved (station: %s)", next.sta_ssid[0] ? "configured" : "none");
    /* The flag is what counts: the task checks it at least once a second, so a
     * full command queue only delays the restart. Several saves merge into one. */
    atomic_store(&s_apply_pending, true);
    const net_cmd_t cmd = CMD_APPLY;
    xQueueSend(s_cmd, &cmd, 0);
    return ESP_OK;
}

/* ---- events -------------------------------------------------------------------- */

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_st.ap_clients++;
        xSemaphoreGive(s_lock);
        ESP_LOGI(TAG, "AP: client joined");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (s_st.ap_clients) s_st.ap_clients--;
        xSemaphoreGive(s_lock);
        ESP_LOGI(TAG, "AP: client left");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *e = data;
        bool was = false;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        was = s_st.sta_connected;
        s_st.sta_connected = false;
        s_st.sta_ip = 0;
        xSemaphoreGive(s_lock);
        if (atomic_load(&s_running)) {
            atomic_store(&s_retry_at, esp_timer_get_time() + APP_WIFI_STA_RETRY_MS * 1000LL);
            ESP_LOGW(TAG, "STA: %s (reason %u), retry in %d s", was ? "disconnected" : "connect failed",
                     e->reason, APP_WIFI_STA_RETRY_MS / 1000);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *e = data;
        wifi_ap_record_t ap = {0};
        esp_wifi_sta_get_ap_info(&ap);
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_st.sta_connected = true;
        s_st.sta_ip = e->ip_info.ip.addr;
        s_st.sta_rssi = ap.rssi;
        s_st.channel = ap.primary;
        xSemaphoreGive(s_lock);
        atomic_store(&s_retry_at, 0);
        ESP_LOGI(TAG, "STA: got IP " IPSTR " (ch%u, %d dBm) -> http://" IPSTR "/ or http://%s.local/",
                 IP2STR(&e->ip_info.ip), ap.primary, ap.rssi, IP2STR(&e->ip_info.ip), APP_WIFI_HOSTNAME);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_LOST_IP) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_st.sta_ip = 0;
        xSemaphoreGive(s_lock);
    }
}

/* ---- bring-up ------------------------------------------------------------------ */

static esp_err_t connect_c6(void)
{
    /* The C6 supply is switched by IO expander 1; esp_hosted resets it again on connect. */
    ESP_RETURN_ON_ERROR(bsp_feature_enable(BSP_FEATURE_WIFI, true), TAG, "C6 power");
    vTaskDelay(pdMS_TO_TICKS(100));
    int64_t t0 = esp_timer_get_time();
    int err = esp_hosted_connect_to_slave();
    ESP_LOGI(TAG, "C6 connect: %s in %" PRId64 " ms", esp_err_to_name(err), (esp_timer_get_time() - t0) / 1000);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }
    esp_hosted_coprocessor_fwver_t ver = {0};
    if (esp_hosted_get_coprocessor_fwversion(&ver) == ESP_OK) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        snprintf(s_st.c6_version, sizeof(s_st.c6_version), "%" PRIu32 ".%" PRIu32 ".%" PRIu32,
                 ver.major1, ver.minor1, ver.patch1);
        xSemaphoreGive(s_lock);
        ESP_LOGI(TAG, "C6 firmware %s", s_st.c6_version);
    }
    return ESP_OK;
}

static esp_err_t init_stack(void)
{
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif");
    esp_err_t err = esp_event_loop_create_default();
    ESP_RETURN_ON_FALSE(err == ESP_OK || err == ESP_ERR_INVALID_STATE, err, TAG, "event loop");
    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif = esp_netif_create_default_wifi_ap();
    ESP_RETURN_ON_FALSE(s_sta_netif && s_ap_netif, ESP_ERR_NO_MEM, TAG, "netif create");
    esp_netif_set_hostname(s_sta_netif, APP_WIFI_HOSTNAME);
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL), TAG, "events");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, on_event, NULL), TAG, "events");

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "esp_wifi_init");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "storage");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_APSTA), TAG, "mode");

    if (mdns_init() == ESP_OK) {
        mdns_hostname_set(APP_WIFI_HOSTNAME);
        mdns_instance_name_set("Tab5 Camera");
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    } else {
        ESP_LOGW(TAG, "mDNS unavailable: use the IP address");
    }
    return ESP_OK;
}

static void apply_config(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    net_settings_t set = s_set;
    xSemaphoreGive(s_lock);

    wifi_config_t ap = {
        .ap = {
            .ssid = APP_WIFI_AP_SSID,
            .ssid_len = sizeof(APP_WIFI_AP_SSID) - 1,
            .channel = APP_WIFI_AP_CHANNEL,
            .max_connection = APP_WIFI_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = { .required = false },
        },
    };
    strlcpy((char *)ap.ap.password, set.ap_pass, sizeof(ap.ap.password));
    esp_err_t err = esp_wifi_set_config(WIFI_IF_AP, &ap);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AP config: %s", esp_err_to_name(err));
    }
    wifi_config_t sta = {0};
    strlcpy((char *)sta.sta.ssid, set.sta_ssid, sizeof(sta.sta.ssid));
    strlcpy((char *)sta.sta.password, set.sta_pass, sizeof(sta.sta.password));
    err = esp_wifi_set_config(WIFI_IF_STA, &sta);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "STA config: %s", esp_err_to_name(err));
    }
    memset(&set, 0, sizeof(set));
}

static esp_err_t start_wifi(void)
{
    apply_config();
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start");
    /* Modem sleep delays incoming TCP ACKs by up to a beacon interval and slows downloads. */
    esp_wifi_set_ps(WIFI_PS_NONE);
    atomic_store(&s_running, true);

    esp_netif_ip_info_t ip = {0};
    esp_netif_get_ip_info(s_ap_netif, &ip);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_st.ap_ip = ip.ip.addr;
    s_st.channel = APP_WIFI_AP_CHANNEL;
    bool sta = s_st.sta_configured;
    xSemaphoreGive(s_lock);
    atomic_store(&s_retry_at, sta ? esp_timer_get_time() : 0);
    ESP_LOGI(TAG, "AP up: SSID %s -> http://" IPSTR "/%s", APP_WIFI_AP_SSID, IP2STR(&ip.ip),
             sta ? ", joining the saved network" : "");
    return ESP_OK;
}

static void stop_wifi(void)
{
    atomic_store(&s_running, false);
    atomic_store(&s_retry_at, 0);
    esp_wifi_stop();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_st.sta_connected = false;
    s_st.sta_ip = 0;
    s_st.ap_clients = 0;
    xSemaphoreGive(s_lock);
}

static void turn_on(void)
{
    if (atomic_load(&s_running)) {
        return;
    }
    set_state(NET_STARTING);
    if (!s_c6_ok) {
        s_c6_ok = connect_c6() == ESP_OK;
        if (!s_c6_ok) {
            ESP_LOGE(TAG, "the C6 did not answer; Wi-Fi stays off");
            set_state(NET_FAILED);
            return;
        }
    }
    if (!s_stack_ok) {
        s_stack_ok = init_stack() == ESP_OK;
        if (!s_stack_ok) {
            set_state(NET_FAILED);
            return;
        }
    }
    if (start_wifi() != ESP_OK) {
        set_state(NET_FAILED);
        return;
    }
    if (web_server_start() != ESP_OK) {
        ESP_LOGE(TAG, "web server did not start");
    }
    set_state(NET_ON);
}

static void turn_off(void)
{
    if (atomic_load(&s_running)) {
        web_server_stop();
        stop_wifi();
        ESP_LOGI(TAG, "Wi-Fi off");
    }
    set_state(NET_OFF);
}

static void net_task(void *arg)
{
    for (;;) {
        net_cmd_t cmd;
        if (xQueueReceive(s_cmd, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
            switch (cmd) {
            case CMD_ON:
                turn_on();
                break;
            case CMD_OFF:
                turn_off();
                break;
            case CMD_APPLY:
                break;      /* only a wake-up; s_apply_pending carries the request */
            }
        }
        if (atomic_load(&s_apply_pending)) {
            if (atomic_load(&s_running)) {
                vTaskDelay(pdMS_TO_TICKS(APP_WIFI_APPLY_DELAY_MS));
                /* Cleared before the restart: a save during it triggers one more. */
                atomic_store(&s_apply_pending, false);
                stop_wifi();
                if (start_wifi() != ESP_OK) {
                    web_server_stop();
                    set_state(NET_FAILED);
                }
            } else {
                atomic_store(&s_apply_pending, false);  /* the next start reads the saved settings */
            }
        }
        int64_t at = atomic_load(&s_retry_at);
        if (at && atomic_load(&s_running) && esp_timer_get_time() >= at) {
            /* Pushed back first so a missing event cannot stop the retries. */
            atomic_store(&s_retry_at, esp_timer_get_time() + APP_WIFI_STA_RETRY_MS * 1000LL);
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "STA connect: %s", esp_err_to_name(err));
            }
        }
    }
}

/* ---- public -------------------------------------------------------------------- */

esp_err_t net_wifi_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS layout changed: erasing it (saved Wi-Fi settings are lost)");
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "NVS erase");
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "NVS");
    s_lock = xSemaphoreCreateMutex();
    s_cmd = xQueueCreate(4, sizeof(net_cmd_t));
    ESP_RETURN_ON_FALSE(s_lock && s_cmd, ESP_ERR_NO_MEM, TAG, "no memory");
    load_settings();
    s_st.state = NET_OFF;
    ESP_RETURN_ON_FALSE(xTaskCreate(net_task, "net_wifi", APP_TASK_NET_STACK, NULL, APP_TASK_NET_PRIO, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task create failed");
    ESP_LOGI(TAG, "ready (off); station %s", s_st.sta_configured ? "configured" : "not configured");
    return ESP_OK;
}

void net_wifi_enable(bool on)
{
    if (!s_cmd) {
        return;
    }
    const net_cmd_t cmd = on ? CMD_ON : CMD_OFF;
    if (xQueueSend(s_cmd, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "busy, request dropped");
    }
}

void net_wifi_get_status(net_status_t *out)
{
    if (!s_lock) {
        memset(out, 0, sizeof(*out));
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_st;
    xSemaphoreGive(s_lock);
}

