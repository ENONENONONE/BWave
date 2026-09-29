#include "bwave_ble.h"
#include "bwave_identity.h"
#include "bwave_csi.h"
#include "bwave_cascade.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "bwave_ble_gatt.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "bwave_ble";

static uint8_t s_mfg_data[16];
static bwave_ble_device_t s_devices[BLE_MAX_DEVICES];
static int s_device_count = 0;
static bool s_scanning = false;

static void start_advertise(void)
{
    struct ble_hs_adv_fields fields = {0};
    struct ble_hs_adv_fields rsp = {0};

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    const uint8_t *hash = bwave_identity_get_hash();

    const bwave_field_state_t *fs = bwave_cascade_get_field();

    s_mfg_data[0]  = 0xFF;
    s_mfg_data[1]  = 0xFF;
    const bwave_self_id_t *self = bwave_identity_self();
    s_mfg_data[2]  = self->position;
    s_mfg_data[3]  = self->element;
    s_mfg_data[4]  = self->fold_element;
    s_mfg_data[5]  = self->ray;
    s_mfg_data[6]  = self->fold_dlog;
    s_mfg_data[7]  = bwave_csi_get_node_id();
    s_mfg_data[8]  = (uint8_t)(fs->xray * 255);
    s_mfg_data[9]  = fs->flip_state;
    s_mfg_data[10] = fs->nano2_active ? (uint8_t)(fs->nano2_pub & 0xFF) : 0;
    s_mfg_data[11] = fs->nano2_active ? (uint8_t)(fs->nano2_fold & 0xFF) : 0;
    memcpy(&s_mfg_data[12], hash, 4);

    fields.mfg_data = s_mfg_data;
    fields.mfg_data_len = sizeof(s_mfg_data);
    ble_gap_adv_set_fields(&fields);

    static const char *name = CONFIG_BWAVE_BLE_NAME;
    rsp.name = (const uint8_t *)name;
    rsp.name_len = (uint8_t)strlen(name);
    rsp.name_is_complete = 1;
    ble_gap_adv_rsp_set_fields(&rsp);

    struct ble_gap_adv_params params = {0};
    params.conn_mode = BLE_GAP_CONN_MODE_NON;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    params.itvl_min = 160;
    params.itvl_max = 800;

    int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL,
                               BLE_HS_FOREVER, &params, NULL, NULL);
    if (rc == 0)
        ESP_LOGI(TAG, "BLE advertising %s pos=%u", name, self->position);
    else if (rc != BLE_HS_EALREADY)
        ESP_LOGE(TAG, "BLE adv start: %d", rc);
}

static int find_or_add_device(const uint8_t *mac)
{
    for (int i = 0; i < s_device_count; i++) {
        if (memcmp(s_devices[i].mac, mac, 6) == 0)
            return i;
    }
    if (s_device_count >= BLE_MAX_DEVICES)
        return -1;
    int idx = s_device_count++;
    memset(&s_devices[idx], 0, sizeof(bwave_ble_device_t));
    memcpy(s_devices[idx].mac, mac, 6);
    return idx;
}

static int scan_event(struct ble_gap_event *event, void *arg)
{
    if (event->type == BLE_GAP_EVENT_DISC) {
        struct ble_gap_disc_desc *desc = &event->disc;
        int idx = find_or_add_device(desc->addr.val);
        if (idx < 0) return 0;

        bwave_ble_device_t *dev = &s_devices[idx];
        dev->rssi = desc->rssi;
        dev->last_seen_ms = (uint32_t)(esp_timer_get_time() / 1000);

        /* capture raw advertisement bytes */
        int raw_n = desc->length_data < 31 ? desc->length_data : 31;
        if (raw_n > 0) {
            memcpy(dev->raw_adv, desc->data, raw_n);
            dev->raw_adv_len = (uint8_t)raw_n;
        }

        struct ble_hs_adv_fields parsed;
        if (ble_hs_adv_parse_fields(&parsed, desc->data,
                                     desc->length_data) == 0) {
            if (parsed.name && parsed.name_len > 0) {
                int n = parsed.name_len < 28 ? parsed.name_len : 28;
                memcpy(dev->name, parsed.name, n);
                dev->name[n] = '\0';
            }
            if (parsed.mfg_data && parsed.mfg_data_len >= 8 &&
                parsed.mfg_data[0] == 0xFF && parsed.mfg_data[1] == 0xFF) {
                dev->is_identity = 1;
                dev->position = parsed.mfg_data[2];
            }

            if (!dev->mirrored && dev->name[0]) {
                dev->mirrored = 1;
                uint16_t pos[29];
                int np = 0;
                for (int k = 0; dev->name[k] && np < 29; k++) {
                    uint16_t p = (uint16_t)(uint8_t)dev->name[k];
                    if (p > 0 && p < GF_P)
                        pos[np++] = p;
                }
                if (np > 0)
                    bwave_cascade_store_device(dev->name, pos, np);

                ESP_LOGI(TAG, "Mirrored %s → %d positions", dev->name, np);
            }

            /* unnamed devices: decompose raw adv bytes through GF(257) */
            if (!dev->mirrored && !dev->name[0] && dev->raw_adv_len > 0) {
                dev->mirrored = 1;
                uint16_t pos[31];
                int np = 0;
                for (int k = 0; k < dev->raw_adv_len && np < 31; k++) {
                    uint16_t p = (uint16_t)dev->raw_adv[k] + 1;
                    if (p > 0 && p < GF_P)
                        pos[np++] = p;
                }
                if (np > 0) {
                    char label[20];
                    snprintf(label, sizeof(label),
                             "%02x%02x%02x%02x%02x%02x",
                             dev->mac[5], dev->mac[4], dev->mac[3],
                             dev->mac[2], dev->mac[1], dev->mac[0]);
                    bwave_cascade_store_device(label, pos, np);
                    ESP_LOGI(TAG, "Decomposed unnamed %s → %d positions",
                             label, np);
                }
            }
        }
        return 0;
    }

    if (event->type == BLE_GAP_EVENT_DISC_COMPLETE) {
        s_scanning = false;
        for (int i = 0; i < s_device_count - 1; i++) {
            for (int j = i + 1; j < s_device_count; j++) {
                if (s_devices[j].rssi > s_devices[i].rssi) {
                    bwave_ble_device_t tmp = s_devices[i];
                    s_devices[i] = s_devices[j];
                    s_devices[j] = tmp;
                }
            }
        }
        ESP_LOGI(TAG, "Scan done — %d devices (sorted by RSSI)", s_device_count);
        start_advertise();
        return 0;
    }

    return 0;
}

void bwave_ble_scan_start(void)
{
    if (s_scanning) return;

    ble_gap_adv_stop();

    struct ble_gap_disc_params params = {0};
    params.passive = 0;
    params.itvl = 160;
    params.window = 80;
    params.filter_duplicates = 1;
    params.limited = 0;

    s_device_count = 0;
    s_scanning = true;

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, 5000, &params,
                          scan_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Scan start failed: %d", rc);
        s_scanning = false;
        start_advertise();
    } else {
        ESP_LOGI(TAG, "Scanning 5s...");
    }
}

int bwave_ble_get_device_count(void)
{
    return s_device_count;
}

const bwave_ble_device_t *bwave_ble_get_devices(void)
{
    return s_devices;
}

void bwave_ble_get_macs_json(char *buf, size_t buflen)
{
    int pos = snprintf(buf, buflen,
        "{\"ok\":true,\"cmd\":\"ble_macs\",\"count\":%d,"
        "\"scanning\":%s,\"devices\":[",
        s_device_count, s_scanning ? "true" : "false");

    for (int i = 0; i < s_device_count && pos < (int)buflen - 200; i++) {
        const bwave_ble_device_t *d = &s_devices[i];
        if (i) buf[pos++] = ',';
        pos += snprintf(buf + pos, buflen - pos,
            "{\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\","
            "\"rssi\":%d,\"name\":\"%s\"",
            d->mac[5], d->mac[4], d->mac[3],
            d->mac[2], d->mac[1], d->mac[0],
            d->rssi, d->name);
        if (d->is_identity)
            pos += snprintf(buf + pos, buflen - pos,
                ",\"identity\":%d", d->position);
        if (d->raw_adv_len > 0) {
            pos += snprintf(buf + pos, buflen - pos,
                ",\"raw_len\":%d,\"raw\":[", d->raw_adv_len);
            for (int j = 0; j < d->raw_adv_len; j++) {
                if (j) buf[pos++] = ',';
                pos += snprintf(buf + pos, buflen - pos,
                    "%u", d->raw_adv[j]);
            }
            buf[pos++] = ']';
            pos += snprintf(buf + pos, buflen - pos,
                ",\"field\":[");
            for (int j = 0; j < d->raw_adv_len; j++) {
                uint16_t fp = (uint16_t)d->raw_adv[j] + 1;
                if (j) buf[pos++] = ',';
                pos += snprintf(buf + pos, buflen - pos,
                    "%u", fp < GF_P ? fp : 0);
            }
            buf[pos++] = ']';
        }
        pos += snprintf(buf + pos, buflen - pos, "}");
    }
    pos += snprintf(buf + pos, buflen - pos, "]}\n");
}

static void on_sync(void)
{
    ESP_LOGI(TAG, "BLE host synced");
    start_advertise();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE reset: %d", reason);
}

static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t bwave_ble_init(void)
{
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    ble_svc_gap_device_name_set(CONFIG_BWAVE_BLE_NAME);
    ble_svc_gap_init();
    bwave_ble_gatt_init();

    nimble_port_freertos_init(host_task);

    ESP_LOGI(TAG, "NimBLE started — advertising + scan ready");
    return ESP_OK;
}
