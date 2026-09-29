/**
 * BWave — ESP32-C6 Brain/Sleep CSI Sensor
 *
 * WiFi 6 CSI capture, vitals extraction (HR, BR, brainwave bands),
 * LCD display, SD card logging, UDP streaming.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_netif_sntp.h"
#include "nvs_flash.h"

#include "bwave_config.h"
#include "bwave_csi.h"
#include "bwave_dsp.h"
#include "bwave_stream.h"
#include "bwave_sd.h"
#include "bwave_lcd.h"
#include "bwave_cmd.h"
#include "bwave_identity.h"
#include "bwave_ble.h"
#include "bwave_cascade.h"
#include "bwave_httpd.h"
#include "bwave_slip.h"

static const char *TAG = "bwave";

bwave_config_t g_bwave_config;

#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

static void event_handler(void *arg, esp_event_base_t base,
                          int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_retry_num++;
        if (s_retry_num % 10 == 0)
            ESP_LOGW(TAG, "WiFi retry %d", s_retry_num);
        vTaskDelay(pdMS_TO_TICKS(s_retry_num > 30 ? 5000 : 1000));
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t inst_any, inst_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &inst_any));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &inst_ip));

    wifi_config_t wifi_cfg = {
        .sta = { .threshold.authmode = WIFI_AUTH_WPA2_PSK },
    };
    strncpy((char *)wifi_cfg.sta.ssid, g_bwave_config.wifi_ssid,
            sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, g_bwave_config.wifi_password,
            sizeof(wifi_cfg.sta.password) - 1);

    if (strlen((char *)wifi_cfg.sta.password) == 0)
        wifi_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));

    /* Per-node MAC from node_id */
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    mac[5] = bwave_csi_get_node_id();
    esp_wifi_set_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "MAC -> %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40);

    ESP_LOGI(TAG, "Connecting to %s", g_bwave_config.wifi_ssid);

    xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));
}

/* LCD + SD logging task */
static void display_task(void *arg)
{
    (void)arg;
    bwave_csi_snapshot_t snap;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));

        float br, hr, delta, theta, alpha;
        bwave_dsp_get_bpm(&br, &hr);
        bwave_dsp_get_brainwave(&delta, &theta, &alpha);
        float motion = bwave_dsp_get_motion();
        bwave_csi_get_snapshot(&snap);

        /* LCD update */
        if (g_bwave_config.lcd_enabled) {
            bwave_lcd_update_body(snap.iq, snap.iq_len);
            bwave_lcd_update(hr, br, delta, theta, alpha,
                             snap.rssi, snap.channel,
                             bwave_csi_get_node_id(),
                             motion, 0);
        }

        /* SD card logging */
        if (g_bwave_config.sd_logging) {
            bwave_sd_log_vitals(bwave_csi_get_node_id(),
                                (int8_t)snap.rssi, hr, br,
                                delta, theta, alpha, motion,
                                snap.iq, snap.iq_len);
        }
    }
}

void app_main(void)
{
    /* NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Config */
    bwave_config_load(&g_bwave_config);
    bwave_csi_set_node_id(g_bwave_config.node_id);

    ESP_LOGI(TAG, "BWave CSI Sensor — Node %u", g_bwave_config.node_id);
    ESP_LOGI(TAG, "  SSID: %s", g_bwave_config.wifi_ssid);
    ESP_LOGI(TAG, "  Target: %s:%d", g_bwave_config.target_ip, g_bwave_config.target_port);
    ESP_LOGI(TAG, "  Edge tier: %u", g_bwave_config.edge_tier);

    /* LCD */
    if (g_bwave_config.lcd_enabled) {
        if (bwave_lcd_init() == ESP_OK)
            bwave_lcd_show_boot_screen();
        else
            ESP_LOGW(TAG, "LCD init failed — continuing without display");
    }

    /* SD card */
    if (g_bwave_config.sd_logging) {
        if (bwave_sd_init() != ESP_OK)
            ESP_LOGW(TAG, "SD init failed — continuing without logging");
    }

    /* WiFi */
    if (g_bwave_config.wifi_ssid[0] == '\0')
        ESP_LOGW(TAG, "No SSID provisioned — set NVS \"bwave\" ssid/password/target_ip");
    wifi_init_sta();

    /* NTP */
    esp_sntp_config_t sntp_cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&sntp_cfg);

    /* HTTP file server (serves /sdcard/www/) */
    bwave_httpd_start();

    /* UDP stream */
    if (bwave_stream_init(g_bwave_config.target_ip, g_bwave_config.target_port) != 0)
        ESP_LOGE(TAG, "UDP init failed");

    /* CSI collection */
    bwave_csi_init();

    /* DSP pipeline */
    bwave_dsp_config_t dsp_cfg = {
        .tier = g_bwave_config.edge_tier,
        .presence_thresh = g_bwave_config.presence_thresh,
        .vital_interval_ms = g_bwave_config.vital_interval_ms,
        .top_k_count = g_bwave_config.top_k_count,
    };
    bwave_dsp_init(&dsp_cfg);

    /* Display + logging task */
    xTaskCreate(display_task, "bwave_ui", 4096, NULL, 3, NULL);

    /* Store own position in mirror */
    {
        uint16_t self_pos = bwave_csi_get_node_id() % GF_P;
        bwave_cascade_store_device("SELF", &self_pos, 1);
    }

    /* Identity */
    bwave_identity_init();

    /* BLE identity beacon */
    bwave_ble_init();

    /* Serial command link over USB */
    bwave_slip_init();

    /* Command handler (now served over UDP via SLIP) */
    bwave_cmd_init();

    ESP_LOGI(TAG, "BWave active — WiFi 6 CSI streaming");

    uint32_t identity_counter = 0;
    char bc_buf[128];
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        if (++identity_counter % 1 == 0) {
            bwave_identity_send();
            bwave_cascade_broadcast_sd(bc_buf, sizeof(bc_buf));
        }
    }
}
