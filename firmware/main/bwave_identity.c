#include "bwave_identity.h"
#include "bwave_csi.h"
#include "bwave_dsp.h"
#include "bwave_stream.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <dirent.h>

static const char *TAG = "bwave_id";

/* Hash starts zeroed; provision via NVS namespace "bwave", key "hash". */
static uint8_t s_hash[32];
static bwave_self_id_t s_self;

const bwave_self_id_t *bwave_identity_self(void)
{
    return &s_self;
}

static uint16_t pow3_mod257(uint16_t e)
{
    uint32_t r = 1, b = 3;
    while (e) {
        if (e & 1) r = (r * b) % 257;
        b = (b * b) % 257;
        e >>= 1;
    }
    return (uint16_t)r;
}

static void derive_self(uint16_t position)
{
    memset(&s_self, 0, sizeof(s_self));
    if (position == 0 || position > 256) return;
    uint16_t el = pow3_mod257(position);
    s_self.position     = (uint8_t)position;
    s_self.element      = (uint8_t)el;
    s_self.fold_element = (uint8_t)pow3_mod257((uint16_t)(256 - position));
    s_self.ray          = (uint8_t)(position % 12);
    s_self.fold_dlog    = (uint8_t)((256 - position) & 0xFF);
}

const uint8_t *bwave_identity_get_hash(void)
{
    return s_hash;
}

static void save_self_to_sd(void)
{
    struct stat st;
    if (stat("/sdcard/ident", &st) != 0)
        mkdir("/sdcard/ident", 0755);

    if (s_self.position == 0) return;

    char path[40];
    snprintf(path, sizeof(path), "/sdcard/ident/%u.json", s_self.position);
    FILE *f = fopen(path, "w");
    if (!f) return;

    fprintf(f, "{\"position\":%u,\"element\":%u,\"fold\":%u,"
            "\"ray\":%u,\"fold_dlog\":%u,"
            "\"hash\":\"", s_self.position, s_self.element,
            s_self.fold_element, s_self.ray, s_self.fold_dlog);
    for (int i = 0; i < 32; i++)
        fprintf(f, "%02x", s_hash[i]);
    fprintf(f, "\",\"node_id\":%u,\"self\":true}\n",
            bwave_csi_get_node_id());
    fclose(f);
    ESP_LOGI(TAG, "Saved self identity to %s", path);
}

void bwave_identity_init(void)
{
    memset(s_hash, 0, sizeof(s_hash));
    uint16_t position = (uint16_t)CONFIG_BWAVE_POSITION;

    nvs_handle_t handle;
    /* DeCLARE fallback: its declared position, trusted only once obs_hash
       exists (DeCLARE writes obs_hash last). "bwave" below overrides it. */
    if (nvs_open("csi_cfg", NVS_READONLY, &handle) == ESP_OK) {
        size_t hlen = 0;
        uint16_t op;
        if (nvs_get_str(handle, "obs_hash", NULL, &hlen) == ESP_OK && hlen > 1 &&
            nvs_get_u16(handle, "obs_pos", &op) == ESP_OK) {
            position = op;
            ESP_LOGI(TAG, "csi_cfg obs_pos=%u", op);
        }
        nvs_close(handle);
    }

    if (nvs_open("bwave", NVS_READONLY, &handle) == ESP_OK) {
        size_t len = 32;
        if (nvs_get_blob(handle, "hash", s_hash, &len) == ESP_OK && len == 32)
            ESP_LOGI(TAG, "NVS hash override loaded");
        uint16_t pv;
        if (nvs_get_u16(handle, "position", &pv) == ESP_OK)
            position = pv;
        nvs_close(handle);
    }

    derive_self(position);

    if (s_self.position == 0) {
        ESP_LOGI(TAG, "Identity unassigned");
    } else {
        ESP_LOGI(TAG, "ID(%u) = %u, fold = %u, ray = %u",
                 s_self.position, s_self.element, s_self.fold_element, s_self.ray);
        ESP_LOGI(TAG, "  %u x %u = 1 (mod 257)", s_self.element, s_self.fold_element);
    }
    ESP_LOGI(TAG, "  packet = %d bytes", (int)sizeof(bwave_identity_pkt_t));

    save_self_to_sd();
}

void bwave_identity_send(void)
{
    bwave_identity_pkt_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.magic = BWAVE_IDENTITY_MAGIC;
    pkt.node_id = bwave_csi_get_node_id();
    pkt.position = s_self.position;
    pkt.element = s_self.element;
    pkt.fold_element = s_self.fold_element;
    pkt.ray = s_self.ray;
    pkt.fold_dlog = s_self.fold_dlog;

    bwave_vitals_pkt_t vitals;
    bool has_vitals = bwave_dsp_get_vitals(&vitals);

    if (has_vitals) {
        pkt.flags = 1;
        pkt.heartrate = (uint16_t)(vitals.heartrate / 100);
        pkt.breathing_rate = vitals.breathing_rate;
        pkt.presence = vitals.presence_score;
        pkt.motion = vitals.motion_energy;
    }

    memcpy(pkt.hash, s_hash, 32);
    pkt.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);

    bwave_csi_snapshot_t snap;
    bwave_csi_get_snapshot(&snap);
    pkt.n_subcarriers = snap.iq_len / 2;

    bwave_stream_send((const uint8_t *)&pkt, sizeof(pkt));
}

void bwave_identity_get_json(char *buf, size_t buflen)
{
    bwave_vitals_pkt_t vitals;
    bool has_vitals = bwave_dsp_get_vitals(&vitals);

    bwave_csi_snapshot_t snap;
    bwave_csi_get_snapshot(&snap);

    int pos = snprintf(buf, buflen,
        "{\"ok\":true,\"cmd\":\"identity\","
        "\"position\":%d,\"element\":%d,\"fold\":%d,"
        "\"ray\":%d,\"fold_dlog\":%d,"
        "\"inverse\":\"%u*%u=1(mod257)\","
        "\"node\":%u,\"n_sc\":%d,"
        "\"hash\":\"",
        s_self.position, s_self.element, s_self.fold_element,
        s_self.ray, s_self.fold_dlog,
        s_self.element, s_self.fold_element,
        bwave_csi_get_node_id(), snap.iq_len / 2);

    for (int i = 0; i < 32; i++)
        pos += snprintf(buf + pos, buflen - pos, "%02x", s_hash[i]);

    pos += snprintf(buf + pos, buflen - pos, "\"");

    if (has_vitals) {
        pos += snprintf(buf + pos, buflen - pos,
            ",\"observer\":{\"hr\":%.2f,\"br\":%.2f,"
            "\"presence\":%.4f,\"motion\":%.4f}",
            vitals.heartrate / 10000.0f,
            vitals.breathing_rate / 100.0f,
            vitals.presence_score,
            vitals.motion_energy);
    }

    pos += snprintf(buf + pos, buflen - pos,
        ",\"pkt_bytes\":%d}\n",
        (int)sizeof(bwave_identity_pkt_t));
}

void bwave_identity_save_to_sd(uint8_t position, uint8_t element,
                               uint8_t fold, uint8_t ray,
                               const uint8_t *hash, const char *genome)
{
    struct stat st;
    if (stat("/sdcard/ident", &st) != 0)
        mkdir("/sdcard/ident", 0755);

    char path[48];
    snprintf(path, sizeof(path), "/sdcard/ident/%d.json", position);

    FILE *f = fopen(path, "w");
    if (!f) {
        ESP_LOGW(TAG, "Cannot write %s", path);
        return;
    }

    fprintf(f, "{\"position\":%d,\"element\":%d,\"fold\":%d,\"ray\":%d,"
            "\"hash\":\"", position, element, fold, ray);
    if (hash)
        for (int i = 0; i < 32; i++)
            fprintf(f, "%02x", hash[i]);
    fprintf(f, "\"");
    if (genome)
        fprintf(f, ",\"genome\":\"%s\"", genome);
    fprintf(f, ",\"self\":false}\n");
    fclose(f);

    ESP_LOGI(TAG, "Saved identity pos=%d to %s", position, path);
}

void bwave_identity_list_sd(char *buf, size_t buflen)
{
    int pos = snprintf(buf, buflen,
        "{\"ok\":true,\"cmd\":\"identities\",\"entries\":[");

    DIR *d = opendir("/sdcard/ident");
    if (!d) {
        pos += snprintf(buf + pos, buflen - pos, "],\"error\":\"no SD\"}\n");
        return;
    }

    struct dirent *ent;
    int count = 0;
    while ((ent = readdir(d)) != NULL) {
        if (ent->d_type != DT_REG) continue;
        char *dot = strrchr(ent->d_name, '.');
        if (!dot || strcmp(dot, ".json") != 0) continue;

        char fpath[300];
        snprintf(fpath, sizeof(fpath), "/sdcard/ident/%.280s", ent->d_name);

        FILE *f = fopen(fpath, "r");
        if (!f) continue;

        char line[512];
        if (fgets(line, sizeof(line), f)) {
            size_t len = strlen(line);
            while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r'))
                line[--len] = '\0';

            if (count > 0) buf[pos++] = ',';
            int remain = buflen - pos - 20;
            if (remain > 0) {
                int n = len < (size_t)remain ? (int)len : remain;
                memcpy(buf + pos, line, n);
                pos += n;
            }
            count++;
        }
        fclose(f);
    }
    closedir(d);

    pos += snprintf(buf + pos, buflen - pos, "],\"count\":%d}\n", count);
}
