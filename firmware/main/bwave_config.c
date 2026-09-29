#include "bwave_config.h"
#include <string.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sdkconfig.h"

static const char *TAG = "bwave_cfg";

static int load_nvs_ns(bwave_config_t *cfg, const char *ns, bool with_node_id)
{
    nvs_handle_t handle;
    if (nvs_open(ns, NVS_READONLY, &handle) != ESP_OK)
        return 0;

    size_t len;
    char buf[BWAVE_PASS_MAX];

    len = sizeof(buf);
    if (nvs_get_str(handle, "ssid", buf, &len) == ESP_OK && len > 1) {
        strncpy(cfg->wifi_ssid, buf, BWAVE_SSID_MAX - 1);
        cfg->wifi_ssid[BWAVE_SSID_MAX - 1] = '\0';
        ESP_LOGI(TAG, "NVS %s ssid=%s", ns, cfg->wifi_ssid);
    }

    len = sizeof(buf);
    if (nvs_get_str(handle, "password", buf, &len) == ESP_OK) {
        strncpy(cfg->wifi_password, buf, BWAVE_PASS_MAX - 1);
        cfg->wifi_password[BWAVE_PASS_MAX - 1] = '\0';
    }

    len = sizeof(buf);
    if (nvs_get_str(handle, "target_ip", buf, &len) == ESP_OK && len > 1) {
        strncpy(cfg->target_ip, buf, BWAVE_IP_MAX - 1);
        cfg->target_ip[BWAVE_IP_MAX - 1] = '\0';
        ESP_LOGI(TAG, "NVS %s target_ip=%s", ns, cfg->target_ip);
    }

    uint16_t port_val;
    if (nvs_get_u16(handle, "target_port", &port_val) == ESP_OK && port_val > 0)
        cfg->target_port = port_val;

    uint8_t u8_val;
    if (with_node_id && nvs_get_u8(handle, "node_id", &u8_val) == ESP_OK)
        cfg->node_id = u8_val;

    nvs_close(handle);
    return 1;
}

void bwave_config_load(bwave_config_t *cfg)
{
    if (!cfg) return;

    strncpy(cfg->wifi_ssid, CONFIG_BWAVE_WIFI_SSID, BWAVE_SSID_MAX - 1);
    cfg->wifi_ssid[BWAVE_SSID_MAX - 1] = '\0';

#ifdef CONFIG_BWAVE_WIFI_PASSWORD
    strncpy(cfg->wifi_password, CONFIG_BWAVE_WIFI_PASSWORD, BWAVE_PASS_MAX - 1);
    cfg->wifi_password[BWAVE_PASS_MAX - 1] = '\0';
#else
    cfg->wifi_password[0] = '\0';
#endif

    strncpy(cfg->target_ip, CONFIG_BWAVE_TARGET_IP, BWAVE_IP_MAX - 1);
    cfg->target_ip[BWAVE_IP_MAX - 1] = '\0';

    cfg->target_port = (uint16_t)CONFIG_BWAVE_TARGET_PORT;
    cfg->wifi_channel = (uint8_t)CONFIG_BWAVE_WIFI_CHANNEL;
    cfg->edge_tier = (uint8_t)CONFIG_BWAVE_EDGE_TIER;
    cfg->vital_interval_ms = (uint16_t)CONFIG_BWAVE_VITAL_INTERVAL_MS;
    cfg->presence_thresh = 0.0f;
    cfg->top_k_count = 8;

#ifdef CONFIG_BWAVE_SD_LOGGING
    cfg->sd_logging = 1;
#else
    cfg->sd_logging = 0;
#endif

#ifdef CONFIG_BWAVE_LCD_ENABLED
    cfg->lcd_enabled = 1;
#else
    cfg->lcd_enabled = 0;
#endif

    /* Node ID: auto from MAC low byte if 0 */
    if (CONFIG_BWAVE_NODE_ID > 0) {
        cfg->node_id = (uint8_t)CONFIG_BWAVE_NODE_ID;
    } else {
        uint8_t mac[6] = {0};
        if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK && mac[5] != 0) {
            cfg->node_id = mac[5];
        } else {
            cfg->node_id = 1;
        }
    }

    /* NVS: DeCLARE's "csi_cfg" first (read-only fallback), then "bwave",
       so a value in "bwave" always wins. node_id is taken from "bwave"
       only; DeCLARE ignores its own NVS node_id and uses the MAC. */
    int found = load_nvs_ns(cfg, "csi_cfg", false);
    found |= load_nvs_ns(cfg, "bwave", true);
    if (!found)
        ESP_LOGI(TAG, "No NVS config, using defaults");
}
