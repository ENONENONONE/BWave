#include "bwave_csi.h"
#include "bwave_config.h"
#include "bwave_stream.h"
#include "bwave_dsp.h"

#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "sdkconfig.h"

extern bwave_config_t g_bwave_config;

static const char *TAG = "bwave_csi";

static uint8_t  s_node_id = 1;
static uint32_t s_sequence = 0;
static uint32_t s_cb_count = 0;
static uint32_t s_send_ok = 0;
static uint32_t s_send_fail = 0;

static int8_t   s_prev_rssi = -127;
#define AGC_JUMP_THRESHOLD 6

#define CSI_MIN_PROCESS_INTERVAL_US  (20 * 1000)
static int64_t s_last_process_us = 0;

#define CSI_MIN_SEND_INTERVAL_US  (20 * 1000)
static int64_t s_last_send_us = 0;

#define CSI_MAX_FRAME_SIZE (CSI_HEADER_SIZE + 4 * 256 * 2)
static uint8_t s_frame_buf[CSI_MAX_FRAME_SIZE];
static uint8_t s_last_rx_mac[6] = {0};

static volatile int  s_snap_rssi = 0;
static volatile int  s_snap_channel = 0;
static int8_t        s_snap_iq[512];
static volatile int  s_snap_iq_len = 0;

#define MAC_TABLE_SIZE 32
static uint8_t s_mac_table[MAC_TABLE_SIZE][6];
static int8_t  s_mac_rssi[MAC_TABLE_SIZE];
static uint16_t s_mac_count[MAC_TABLE_SIZE];
static int     s_mac_table_len = 0;

static esp_timer_handle_t s_inject_timer = NULL;
static int64_t s_inject_backoff_until_us = 0;

void bwave_csi_set_node_id(uint8_t node_id)
{
    s_node_id = node_id;
    ESP_LOGI(TAG, "node_id=%u", (unsigned)node_id);
}

uint8_t bwave_csi_get_node_id(void)
{
    return s_node_id;
}

static size_t serialize_frame(const wifi_csi_info_t *info, uint8_t *buf, size_t buf_len)
{
    if (!info || !buf || !info->buf) return 0;

    uint16_t iq_len = (uint16_t)info->len;
    uint16_t n_sub = iq_len / 2;
    size_t frame_size = CSI_HEADER_SIZE + iq_len;
    if (frame_size > buf_len) return 0;

    uint8_t channel = info->rx_ctrl.channel;
    uint32_t freq_mhz;
    if (channel >= 1 && channel <= 13)
        freq_mhz = 2412 + (channel - 1) * 5;
    else if (channel == 14)
        freq_mhz = 2484;
    else
        freq_mhz = 0;

    uint32_t magic = CSI_MAGIC;
    memcpy(&buf[0], &magic, 4);
    buf[4] = s_node_id;
    buf[5] = 1;
    memcpy(&buf[6], &n_sub, 2);
    memcpy(&buf[8], &freq_mhz, 4);
    uint32_t seq = s_sequence++;
    memcpy(&buf[12], &seq, 4);
    buf[16] = (uint8_t)(int8_t)info->rx_ctrl.rssi;
    buf[17] = (uint8_t)(int8_t)info->rx_ctrl.noise_floor;
    buf[18] = 2;

    int8_t cur_rssi = (int8_t)info->rx_ctrl.rssi;
    uint8_t agc = 0;
    if (s_prev_rssi != -127) {
        int diff = (int)cur_rssi - (int)s_prev_rssi;
        if (diff < 0) diff = -diff;
        if (diff > AGC_JUMP_THRESHOLD) agc = 1;
    }
    s_prev_rssi = cur_rssi;
#if CONFIG_SOC_WIFI_HE_SUPPORT
    uint8_t secondary = info->rx_ctrl.second & 0x03;
    buf[19] = (secondary ? 1 : 0) | (agc << 3) | (1 << 4);
#else
    uint8_t cwb = info->rx_ctrl.cwb & 0x01;
    uint8_t sm  = info->rx_ctrl.sig_mode & 0x03;
    buf[19] = cwb | (sm << 1) | (agc << 3);
#endif

    memcpy(&buf[20], s_last_rx_mac, 6);
    memcpy(&buf[CSI_HEADER_SIZE], info->buf, iq_len);

    return frame_size;
}

static void wifi_csi_callback(void *ctx, wifi_csi_info_t *info)
{
    (void)ctx;

    int64_t now_us = esp_timer_get_time();
    if ((now_us - s_last_process_us) < CSI_MIN_PROCESS_INTERVAL_US)
        return;
    s_last_process_us = now_us;

    s_cb_count++;

    if (s_cb_count <= 3 || (s_cb_count % 100) == 0) {
        ESP_LOGI(TAG, "CSI #%lu: len=%d rssi=%d ch=%d src=%02x:%02x:%02x:%02x:%02x:%02x",
                 (unsigned long)s_cb_count, info->len,
                 info->rx_ctrl.rssi, info->rx_ctrl.channel,
                 s_last_rx_mac[0], s_last_rx_mac[1], s_last_rx_mac[2],
                 s_last_rx_mac[3], s_last_rx_mac[4], s_last_rx_mac[5]);
    }

    s_snap_rssi = info->rx_ctrl.rssi;
    s_snap_channel = info->rx_ctrl.channel;
    if (info->buf && info->len > 0) {
        int copy = info->len < 512 ? info->len : 512;
        memcpy(s_snap_iq, info->buf, copy);
        s_snap_iq_len = copy;
    }

    /* Track unique MACs */
    if (s_last_rx_mac[0] | s_last_rx_mac[1] | s_last_rx_mac[2] |
        s_last_rx_mac[3] | s_last_rx_mac[4] | s_last_rx_mac[5]) {
        int found = -1;
        for (int i = 0; i < s_mac_table_len; i++) {
            if (memcmp(s_mac_table[i], s_last_rx_mac, 6) == 0) {
                found = i;
                break;
            }
        }
        if (found >= 0) {
            s_mac_count[found]++;
            s_mac_rssi[found] = (int8_t)info->rx_ctrl.rssi;
        } else if (s_mac_table_len < MAC_TABLE_SIZE) {
            memcpy(s_mac_table[s_mac_table_len], s_last_rx_mac, 6);
            s_mac_rssi[s_mac_table_len] = (int8_t)info->rx_ctrl.rssi;
            s_mac_count[s_mac_table_len] = 1;
            s_mac_table_len++;
            ESP_LOGI(TAG, "NEW device #%d: %02x:%02x:%02x:%02x:%02x:%02x rssi=%d",
                     s_mac_table_len,
                     s_last_rx_mac[0], s_last_rx_mac[1], s_last_rx_mac[2],
                     s_last_rx_mac[3], s_last_rx_mac[4], s_last_rx_mac[5],
                     (int)info->rx_ctrl.rssi);
        }
    }

    /* Dump MAC table every 500 callbacks */
    if ((s_cb_count % 500) == 0 && s_mac_table_len > 0) {
        ESP_LOGI(TAG, "--- Devices seen: %d ---", s_mac_table_len);
        for (int i = 0; i < s_mac_table_len; i++) {
            ESP_LOGI(TAG, "  [%d] %02x:%02x:%02x:%02x:%02x:%02x  cnt=%u rssi=%d",
                     i, s_mac_table[i][0], s_mac_table[i][1], s_mac_table[i][2],
                     s_mac_table[i][3], s_mac_table[i][4], s_mac_table[i][5],
                     (unsigned)s_mac_count[i], (int)s_mac_rssi[i]);
        }
    }

    size_t frame_len = serialize_frame(info, s_frame_buf, sizeof(s_frame_buf));

    if (frame_len > 0) {
        int64_t now = esp_timer_get_time();
        if ((now - s_last_send_us) >= CSI_MIN_SEND_INTERVAL_US) {
            int ret = bwave_stream_send(s_frame_buf, frame_len);
            if (ret > 0) {
                s_send_ok++;
                s_last_send_us = now;
            } else {
                s_send_fail++;
            }
        }
    }

    if (info->buf && info->len > 0) {
        bwave_dsp_enqueue((const uint8_t *)info->buf, (uint16_t)info->len,
                          (int8_t)info->rx_ctrl.rssi, info->rx_ctrl.channel);
    }
}

static void wifi_promiscuous_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (!buf || (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA)) return;
    const wifi_promiscuous_pkt_t *p = (const wifi_promiscuous_pkt_t *)buf;
#if !CONFIG_SOC_WIFI_HE_SUPPORT
    if (p->rx_ctrl.sig_len < 16) return;
#endif
    memcpy(s_last_rx_mac, p->payload + 10, 6);
}

static void inject_timer_cb(void *arg)
{
    (void)arg;
    if (s_inject_backoff_until_us > 0) {
        if (esp_timer_get_time() < s_inject_backoff_until_us) return;
        s_inject_backoff_until_us = 0;
    }
    esp_err_t err = bwave_csi_inject_ndp();
    if (err == ESP_ERR_NO_MEM)
        s_inject_backoff_until_us = esp_timer_get_time() + 200000;
}

esp_err_t bwave_csi_inject_ndp(void)
{
    uint8_t ndp[24];
    memset(ndp, 0, sizeof(ndp));
    ndp[0] = 0x40;
    memset(&ndp[4], 0xFF, 6);
    ndp[10] = 0x02; ndp[11] = 0x57; ndp[12] = 0x42;
    ndp[13] = 0x00; ndp[14] = 0x00; ndp[15] = s_node_id;
    memset(&ndp[16], 0xFF, 6);

    return esp_wifi_80211_tx(WIFI_IF_STA, ndp, sizeof(ndp), false);
}

void bwave_csi_init(void)
{
    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_cb));

    wifi_promiscuous_filter_t filt = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA,
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filt));

    /* WiFi 6 HE CSI configuration */
#if CONFIG_SOC_WIFI_HE_SUPPORT
    wifi_csi_config_t csi_cfg;
    memset(&csi_cfg, 0, sizeof(csi_cfg));
    csi_cfg.enable = 1U;
    csi_cfg.acquire_csi_legacy = 1U;
    csi_cfg.acquire_csi_ht20 = 1U;
    csi_cfg.acquire_csi_ht40 = 1U;
    csi_cfg.acquire_csi_su = 1U;
    csi_cfg.acquire_csi_mu = 1U;
    csi_cfg.acquire_csi_dcm = 1U;
    csi_cfg.acquire_csi_beamformed = 1U;
#if CONFIG_SOC_WIFI_MAC_VERSION_NUM >= 3
    csi_cfg.acquire_csi_force_lltf = 1U;
    csi_cfg.acquire_csi_vht = 1U;
    csi_cfg.acquire_csi_he_stbc_mode = ESP_CSI_ACQUIRE_STBC_SAMPLE_HELTFS;
    csi_cfg.val_scale_cfg = 0U;
#else
    csi_cfg.acquire_csi_he_stbc = ESP_CSI_ACQUIRE_STBC_SAMPLE_HELTFS;
    csi_cfg.val_scale_cfg = 0U;
#endif
    csi_cfg.dump_ack_en = 0U;
#else
    wifi_csi_config_t csi_cfg = {
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = true,
        .ltf_merge_en = true,
        .channel_filter_en = false,
        .manu_scale = false,
        .shift = false,
    };
#endif

    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_callback, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));

    /* NDP injection at 20 Hz */
    const esp_timer_create_args_t inj_args = {
        .callback = &inject_timer_cb,
        .name = "bwave_ndp",
    };
    if (esp_timer_create(&inj_args, &s_inject_timer) == ESP_OK) {
        esp_timer_start_periodic(s_inject_timer, 50000);
        ESP_LOGI(TAG, "NDP injection 20 Hz");
    }

    ESP_LOGI(TAG, "CSI initialized (node_id=%u, WiFi 6 HE=%s)",
             (unsigned)s_node_id,
#if CONFIG_SOC_WIFI_HE_SUPPORT
             "yes"
#else
             "no"
#endif
    );
}

void bwave_csi_get_snapshot(bwave_csi_snapshot_t *out)
{
    out->rssi      = s_snap_rssi;
    out->channel   = s_snap_channel;
    out->cb_count  = s_cb_count;
    out->send_ok   = s_send_ok;
    out->send_fail = s_send_fail;
    out->iq_len    = s_snap_iq_len;
    if (s_snap_iq_len > 0)
        memcpy(out->iq, s_snap_iq, s_snap_iq_len);
}

int bwave_csi_get_mac_table(uint8_t macs[][6], int8_t *rssi, uint16_t *counts, int max)
{
    int n = s_mac_table_len < max ? s_mac_table_len : max;
    for (int i = 0; i < n; i++) {
        memcpy(macs[i], s_mac_table[i], 6);
        if (rssi) rssi[i] = s_mac_rssi[i];
        if (counts) counts[i] = s_mac_count[i];
    }
    return n;
}
