#include <string.h>
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "lwip/def.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "nvs.h"
#include "app_config.h"
#include "m5bus.h"
#include "net_eth.h"
#include "web_server.h"

static const char *TAG = "net_eth";

#define NVS_NS  "eth"

static esp_netif_t *s_netif;
static esp_eth_handle_t s_eth;
static TimerHandle_t s_apply_timer;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static eth_status_t s_st;       /* guarded by s_mux */

/* ---- settings ------------------------------------------------------------------ */

static void load_settings(void)
{
    s_st.dhcp = true;
    s_st.cfg_ip = ESP_IP4TOADDR(192, 168, 1, 100);
    s_st.cfg_mask = ESP_IP4TOADDR(255, 255, 255, 0);
    s_st.cfg_gw = ESP_IP4TOADDR(192, 168, 1, 1);
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        uint8_t dhcp;
        if (nvs_get_u8(h, "dhcp", &dhcp) == ESP_OK) {
            s_st.dhcp = dhcp != 0;
        }
        nvs_get_u32(h, "ip", &s_st.cfg_ip);
        nvs_get_u32(h, "mask", &s_st.cfg_mask);
        nvs_get_u32(h, "gw", &s_st.cfg_gw);
        nvs_close(h);
    }
}

/* Valid netmask: contiguous ones from the top, at least /8 and at most /30. */
static bool mask_ok(uint32_t mask_lwip)
{
    uint32_t m = lwip_ntohl(mask_lwip);
    uint32_t inv = ~m;
    return (inv & (inv + 1)) == 0 && m >= 0xff000000u && m <= 0xfffffffcu;
}

static bool settings_ok(uint32_t ip, uint32_t mask, uint32_t gw)
{
    if (!mask_ok(mask)) {
        return false;
    }
    uint32_t host = lwip_ntohl(ip) & ~lwip_ntohl(mask);
    uint32_t first = lwip_ntohl(ip) >> 24;
    if (host == 0 || host == ~lwip_ntohl(mask) || first == 0 || first == 127 || first >= 224) {
        return false;       /* network or broadcast address, or not a unicast address */
    }
    if (gw != 0 && ((gw & mask) != (ip & mask) || gw == ip)) {
        return false;       /* the gateway must be another address on the same subnet */
    }
    return true;
}

/* Put the saved mode / address on the interface (any task). */
static void apply_settings(void)
{
    taskENTER_CRITICAL(&s_mux);
    const bool dhcp = s_st.dhcp;
    esp_netif_ip_info_t info = {
        .ip.addr = s_st.cfg_ip, .netmask.addr = s_st.cfg_mask, .gw.addr = s_st.cfg_gw,
    };
    taskEXIT_CRITICAL(&s_mux);
    if (dhcp) {
        esp_err_t err = esp_netif_dhcpc_start(s_netif);
        if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
            ESP_LOGE(TAG, "DHCP start: %s", esp_err_to_name(err));
        }
        ESP_LOGI(TAG, "mode: DHCP");
    } else {
        esp_err_t err = esp_netif_dhcpc_stop(s_netif);
        if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
            ESP_LOGE(TAG, "DHCP stop: %s", esp_err_to_name(err));
        }
        err = esp_netif_set_ip_info(s_netif, &info);
        ESP_LOGI(TAG, "mode: fixed " IPSTR " / " IPSTR " gw " IPSTR " (%s)", IP2STR(&info.ip), IP2STR(&info.netmask),
                 IP2STR(&info.gw), esp_err_to_name(err));
    }
}

static void on_apply_timer(TimerHandle_t t)
{
    apply_settings();
}

esp_err_t net_eth_save_settings(bool dhcp, uint32_t ip, uint32_t mask, uint32_t gw)
{
    ESP_RETURN_ON_FALSE(s_netif, ESP_ERR_INVALID_STATE, TAG, "no LAN module");
    if (!dhcp && !settings_ok(ip, mask, gw)) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_u8(h, "dhcp", dhcp);
        if (err == ESP_OK && !dhcp) err = nvs_set_u32(h, "ip", ip);
        if (err == ESP_OK && !dhcp) err = nvs_set_u32(h, "mask", mask);
        if (err == ESP_OK && !dhcp) err = nvs_set_u32(h, "gw", gw);
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
    }
    ESP_RETURN_ON_ERROR(err, TAG, "settings not saved");
    taskENTER_CRITICAL(&s_mux);
    s_st.dhcp = dhcp;
    if (!dhcp) {
        s_st.cfg_ip = ip;
        s_st.cfg_mask = mask;
        s_st.cfg_gw = gw;
    }
    taskEXIT_CRITICAL(&s_mux);
    xTimerReset(s_apply_timer, 0);      /* applied a moment later, after the reply */
    return ESP_OK;
}

/* ---- events -------------------------------------------------------------------- */

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == ETH_EVENT && id == ETHERNET_EVENT_CONNECTED) {
        taskENTER_CRITICAL(&s_mux);
        s_st.link = true;
        taskEXIT_CRITICAL(&s_mux);
        ESP_LOGI(TAG, "link up");
    } else if (base == ETH_EVENT && id == ETHERNET_EVENT_DISCONNECTED) {
        taskENTER_CRITICAL(&s_mux);
        s_st.link = false;
        taskEXIT_CRITICAL(&s_mux);
        ESP_LOGW(TAG, "link down");
    } else if (base == IP_EVENT && id == IP_EVENT_ETH_GOT_IP) {
        const ip_event_got_ip_t *e = data;
        ESP_LOGI(TAG, "address " IPSTR " -> http://" IPSTR "/", IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.ip));
    }
}

/* ---- init ---------------------------------------------------------------------- */

esp_err_t net_eth_init(void)
{
    load_settings();
    m5bus_power_on();

    const spi_bus_config_t bus = {
        .mosi_io_num = M5BUS_MOSI,
        .miso_io_num = M5BUS_MISO,
        .sclk_io_num = M5BUS_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(M5BUS_SPI_HOST, &bus, SPI_DMA_CH_AUTO), TAG, "SPI bus");
    spi_device_interface_config_t dev = {
        .mode = 0,
        .clock_speed_hz = APP_ETH_SPI_MHZ * 1000 * 1000,
        .queue_size = 20,
        .spics_io_num = M5BUS_LAN_CS,
    };
    eth_w5500_config_t w5500 = ETH_W5500_DEFAULT_CONFIG(M5BUS_SPI_HOST, &dev);
    w5500.int_gpio_num = -1;                /* INT jumper position unknown: poll */
    w5500.poll_period_ms = APP_ETH_POLL_MS;
    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    mac_cfg.rx_task_stack_size = 4096;
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.reset_gpio_num = -1;            /* RST jumper position unknown: soft reset */
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500, &mac_cfg);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_cfg);
    if (!mac || !phy) {
        spi_bus_free(M5BUS_SPI_HOST);
        return ESP_ERR_NO_MEM;
    }
    esp_eth_config_t cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_err_t err = esp_eth_driver_install(&cfg, &s_eth);
    if (err != ESP_OK) {
        /* No W5500 behind the chip select: no LAN module on the bus. */
        mac->del(mac);
        phy->del(phy);
        spi_bus_free(M5BUS_SPI_HOST);
        ESP_LOGI(TAG, "no LAN module (%s)", esp_err_to_name(err));
        return ESP_ERR_NOT_FOUND;
    }
    uint8_t addr[6];
    ESP_ERROR_CHECK(esp_read_mac(addr, ESP_MAC_ETH));
    esp_eth_ioctl(s_eth, ETH_CMD_S_MAC_ADDR, addr);

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif");
    err = esp_event_loop_create_default();
    ESP_RETURN_ON_FALSE(err == ESP_OK || err == ESP_ERR_INVALID_STATE, err, TAG, "event loop");
    const esp_netif_config_t ncfg = ESP_NETIF_DEFAULT_ETH();
    s_netif = esp_netif_new(&ncfg);
    ESP_RETURN_ON_FALSE(s_netif, ESP_ERR_NO_MEM, TAG, "netif");
    esp_netif_set_hostname(s_netif, APP_WIFI_HOSTNAME);
    ESP_RETURN_ON_ERROR(esp_netif_attach(s_netif, esp_eth_new_netif_glue(s_eth)), TAG, "attach");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, on_event, NULL), TAG, "events");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, on_event, NULL), TAG, "events");

    s_apply_timer = xTimerCreate("eth_apply", pdMS_TO_TICKS(APP_ETH_APPLY_DELAY_MS), pdFALSE, NULL, on_apply_timer);
    ESP_RETURN_ON_FALSE(s_apply_timer, ESP_ERR_NO_MEM, TAG, "timer");
    taskENTER_CRITICAL(&s_mux);
    s_st.present = true;
    memcpy(s_st.mac, addr, sizeof(addr));
    const bool fixed = !s_st.dhcp;
    taskEXIT_CRITICAL(&s_mux);
    if (fixed) {
        apply_settings();       /* the default netif runs the DHCP client: switch it off first */
    }
    ESP_RETURN_ON_ERROR(esp_eth_start(s_eth), TAG, "start");
    /* The page serves on every interface; keep it up for the wired side. */
    if (web_server_start(WEB_USER_ETH) != ESP_OK) {
        ESP_LOGE(TAG, "web server did not start");
    }
    ESP_LOGI(TAG, "W5500 up, MAC %02x:%02x:%02x:%02x:%02x:%02x, SPI %d MHz, poll %d ms", addr[0], addr[1], addr[2],
             addr[3], addr[4], addr[5], APP_ETH_SPI_MHZ, APP_ETH_POLL_MS);
    return ESP_OK;
}

void net_eth_get_status(eth_status_t *out)
{
    taskENTER_CRITICAL(&s_mux);
    *out = s_st;
    taskEXIT_CRITICAL(&s_mux);
    if (out->present && s_netif) {
        esp_netif_ip_info_t info = {0};
        if (esp_netif_get_ip_info(s_netif, &info) == ESP_OK) {
            out->ip = info.ip.addr;
            out->mask = info.netmask.addr;
            out->gw = info.gw.addr;
        }
    }
}
