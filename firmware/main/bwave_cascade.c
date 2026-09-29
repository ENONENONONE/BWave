#include "bwave_cascade.h"
#include "bwave_identity.h"
#include "bwave_stream.h"
#include "bwave_csi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static const char *TAG = "bwave_cascade";

static uint16_t s_dlog[GF_P];
static uint16_t s_inv[GF_P];
static bool s_init = false;
static int s_last_bc_count = 0;

static bwave_field_state_t s_field;

static uint8_t s_mirror[32];

typedef struct {
    char     name[29];
    uint16_t positions[MAX_DEVICE_POS];
    int      npos;
} mirror_dev_t;

static mirror_dev_t s_devices[MAX_MIRROR_DEVICES];
static int s_ndev = 0;

static const uint8_t PI_DIGIT[257] = {
    3,1,4,1,5,9,2,6,5,3,5,8,9,7,9,3,2,3,8,4,
    6,2,6,4,3,3,8,3,2,7,9,5,0,2,8,8,4,1,9,7,
    1,6,9,3,9,9,3,7,5,1,0,5,8,2,0,9,7,4,9,4,
    4,5,9,2,3,0,7,8,1,6,4,0,6,2,8,6,2,0,8,9,
    9,8,6,2,8,0,3,4,8,2,5,3,4,2,1,1,7,0,6,7,
    9,8,2,1,4,8,0,8,6,5,1,3,2,8,2,3,0,6,6,4,
    7,0,9,3,8,4,4,6,0,9,5,5,0,5,8,2,2,3,1,7,
    2,5,3,5,9,4,0,8,1,2,8,4,8,1,1,1,7,4,5,0,
    2,8,4,1,0,2,7,0,1,9,3,8,5,2,1,1,0,5,5,5,
    9,6,4,4,6,2,2,9,4,8,9,5,4,9,3,0,3,8,1,9,
    6,4,4,2,8,8,1,0,9,7,5,6,6,5,9,3,3,4,4,6,
    1,2,8,4,7,5,6,4,8,2,3,3,7,8,6,7,8,3,1,6,
    5,2,7,1,2,0,1,9,0,9,1,4,5,6,4,8,5
};

static const char *OP_NAME[] = {
    "NOP","IDENT","EXIST","SEED","SHIFT","STORE","FOLD","JUMP","SYNC","HALT"
};

static uint16_t pow_mod_257(uint32_t base, uint32_t exp)
{
    uint32_t result = 1;
    base %= 257;
    while (exp > 0) {
        if (exp & 1) result = (result * base) % 257;
        base = (base * base) % 257;
        exp >>= 1;
    }
    return (uint16_t)result;
}

/* ── NVS device storage ─────────────────────────────────────── */

static void rebuild_bitmap(void)
{
    memset(s_mirror, 0, sizeof(s_mirror));
    for (int d = 0; d < s_ndev; d++) {
        for (int i = 0; i < s_devices[d].npos; i++) {
            uint16_t p = s_devices[d].positions[i];
            if (p > 0 && p < GF_P)
                s_mirror[p / 8] |= (1 << (p % 8));
        }
    }
}

static void devices_save(void)
{
    nvs_handle_t h;
    if (nvs_open("cascade", NVS_READWRITE, &h) != ESP_OK) return;

    nvs_set_u8(h, "ndev", (uint8_t)s_ndev);

    for (int d = 0; d < s_ndev; d++) {
        uint8_t blob[128];
        int name_len = (int)strlen(s_devices[d].name);
        int npos = s_devices[d].npos;
        int off = 0;

        blob[off++] = (uint8_t)name_len;
        memcpy(blob + off, s_devices[d].name, name_len);
        off += name_len;
        blob[off++] = (uint8_t)npos;
        for (int i = 0; i < npos; i++) {
            blob[off++] = (uint8_t)(s_devices[d].positions[i] & 0xFF);
            blob[off++] = (uint8_t)(s_devices[d].positions[i] >> 8);
        }

        char key[8];
        snprintf(key, sizeof(key), "d%u", (unsigned)d);
        nvs_set_blob(h, key, blob, off);
    }

    for (int d = s_ndev; d < MAX_MIRROR_DEVICES; d++) {
        char key[8];
        snprintf(key, sizeof(key), "d%u", (unsigned)d);
        nvs_erase_key(h, key);
    }

    rebuild_bitmap();
    nvs_set_blob(h, "mirror", s_mirror, sizeof(s_mirror));
    nvs_commit(h);
    nvs_close(h);
}

static void devices_load(void)
{
    nvs_handle_t h;
    if (nvs_open("cascade", NVS_READONLY, &h) != ESP_OK) return;

    uint8_t ndev = 0;
    if (nvs_get_u8(h, "ndev", &ndev) != ESP_OK) {
        size_t len = sizeof(s_mirror);
        if (nvs_get_blob(h, "mirror", s_mirror, &len) == ESP_OK) {
            ESP_LOGI(TAG, "Legacy bitmap loaded, no devices");
        }
        nvs_close(h);
        return;
    }

    s_ndev = (ndev > MAX_MIRROR_DEVICES) ? MAX_MIRROR_DEVICES : ndev;

    for (int d = 0; d < s_ndev; d++) {
        char key[8];
        snprintf(key, sizeof(key), "d%u", (unsigned)d);

        uint8_t blob[128];
        size_t len = sizeof(blob);
        if (nvs_get_blob(h, key, blob, &len) != ESP_OK || len < 2) {
            s_devices[d].name[0] = '\0';
            s_devices[d].npos = 0;
            continue;
        }

        int off = 0;
        int name_len = blob[off++];
        if (name_len > 28) name_len = 28;
        memcpy(s_devices[d].name, blob + off, name_len);
        s_devices[d].name[name_len] = '\0';
        off += name_len;

        int npos = blob[off++];
        if (npos > MAX_DEVICE_POS) npos = MAX_DEVICE_POS;
        s_devices[d].npos = npos;
        for (int i = 0; i < npos; i++) {
            s_devices[d].positions[i] = blob[off] | ((uint16_t)blob[off + 1] << 8);
            off += 2;
        }
    }

    nvs_close(h);
    rebuild_bitmap();
}

static void ensure_init(void)
{
    if (s_init) return;
    uint32_t val = 1;
    for (int i = 0; i < 256; i++) {
        s_dlog[val] = (uint16_t)i;
        val = (val * GF_G) % GF_P;
    }
    s_dlog[0] = 0;

    s_inv[0] = 0;
    for (int p = 1; p < GF_P; p++)
        s_inv[p] = pow_mod_257(p, 255);

    memset(s_mirror, 0, sizeof(s_mirror));
    s_ndev = 0;
    memset(s_devices, 0, sizeof(s_devices));
    memset(&s_field, 0, sizeof(s_field));
    devices_load();
    s_init = true;

    int count = 0;
    for (int i = 0; i < 32; i++)
        for (int b = 0; b < 8; b++)
            if (s_mirror[i] & (1 << b)) count++;
    ESP_LOGI(TAG, "Mirror loaded: %d devices, %d positions", s_ndev, count);
}

/* ── JSON helpers ───────────────────────────────────────────── */

static int j_get_int(const char *json, const char *key, int *out)
{
    char needle[48];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *p = strstr(json, needle);
    if (!p) return 0;
    p = strchr(p + strlen(needle), ':');
    if (!p) return 0;
    p++;
    while (*p == ' ') p++;
    *out = atoi(p);
    return 1;
}

static int j_get_str(const char *json, const char *key, char *out, int maxlen)
{
    char needle[48];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *p = strstr(json, needle);
    if (!p) return 0;
    p = strchr(p + strlen(needle), ':');
    if (!p) return 0;
    p = strchr(p, '"');
    if (!p) return 0;
    p++;
    int i = 0;
    while (*p && *p != '"' && i < maxlen - 1)
        out[i++] = *p++;
    out[i] = 0;
    return 1;
}

static int j_get_int_array(const char *json, const char *key,
                           uint16_t *arr, int maxn)
{
    char needle[48];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *p = strstr(json, needle);
    if (!p) return 0;
    p = strchr(p + strlen(needle), '[');
    if (!p) return 0;
    p++;
    int count = 0;
    while (*p && *p != ']' && count < maxn) {
        while (*p == ' ' || *p == ',') p++;
        if (*p == ']') break;
        arr[count++] = (uint16_t)atoi(p);
        while (*p && *p != ',' && *p != ']') p++;
    }
    return count;
}

/* ── GF(257) element operations ─────────────────────────────── */

uint16_t bwave_cascade_dlog(uint16_t pos)
{
    ensure_init();
    if (pos == 0 || pos >= GF_P) return 0;
    return s_dlog[pos];
}

bool bwave_cascade_is_qr(uint16_t pos)
{
    if (pos == 0 || pos >= GF_P) return false;
    return (bwave_cascade_dlog(pos) % 2) == 0;
}

uint16_t bwave_cascade_inv(uint16_t p)
{
    ensure_init();
    if (p == 0 || p >= GF_P) return 0;
    return s_inv[p];
}

uint16_t bwave_cascade_neg(uint16_t p)
{
    if (p == 0 || p >= GF_P) return 0;
    return (uint16_t)(GF_P - p);
}

uint16_t bwave_cascade_ray(uint16_t p)
{
    return (uint16_t)(bwave_cascade_dlog(p) % 12);
}

/* ── GF(257^2) extension field: omega^2 = 3 (QNR) ──────────── */

uint16_t bwave_cascade_norm(uint16_t a, uint16_t b)
{
    uint32_t aa = ((uint32_t)a * a) % GF_P;
    uint32_t bb3 = (3 * ((uint32_t)b * b) % GF_P) % GF_P;
    return (uint16_t)((aa + GF_P - bb3) % GF_P);
}

uint16_t bwave_cascade_trace(uint16_t a, uint16_t b)
{
    (void)b;
    return (uint16_t)((2 * (uint32_t)a) % GF_P);
}

/* ── derived operations ─────────────────────────────────────── */

uint16_t bwave_cascade_quality(uint16_t p)
{
    ensure_init();
    if (p == 0 || p >= GF_P) return 0;
    uint16_t fold = (uint16_t)(256 - p);
    if (fold == 0) return 0;
    return (uint16_t)(((uint32_t)p * s_inv[fold]) % GF_P);
}

uint16_t bwave_cascade_impedance(uint16_t p)
{
    if (p == 0 || p >= GF_P) return 0;
    return (uint16_t)(((uint32_t)p * 120) % GF_P);
}

uint16_t bwave_cascade_reflection(uint16_t p)
{
    ensure_init();
    if (p == 0 || p >= GF_P) return 0;
    uint16_t num = (uint16_t)((p + GF_P - 120) % GF_P);
    uint16_t den = (uint16_t)((p + 120) % GF_P);
    if (den == 0) return 0;
    return (uint16_t)(((uint32_t)num * s_inv[den]) % GF_P);
}

/* ── single-position cascade lookup (read-only, no store) ───── */

void bwave_cascade_handle(const char *json, char *buf, size_t buflen)
{
    int pos = -1;
    if (!j_get_int(json, "pos", &pos) || pos < 0 || pos > 256) {
        snprintf(buf, buflen,
            "{\"ok\":false,\"cmd\":\"cascade\",\"err\":\"pos must be 0-256\"}\n");
        return;
    }

    ensure_init();

    uint16_t p = (uint16_t)pos;
    uint16_t dl = (p > 0) ? s_dlog[p] : 0;
    bool qr = (p > 0) ? ((dl % 2) == 0) : false;
    uint16_t ray = dl % 12;
    uint16_t depth = dl / 12;
    int fold = 256 - pos;
    int mod210 = pos % 210;
    int pi = PI_DIGIT[pos];
    uint16_t neg = bwave_cascade_neg(p);
    uint16_t inv = (p > 0) ? s_inv[p] : 0;
    uint16_t fold_u = (fold > 0 && fold < 257) ? (uint16_t)fold : 0;
    uint16_t norm = bwave_cascade_norm(p, fold_u);
    uint16_t trace = bwave_cascade_trace(p, fold_u);
    uint16_t quality = bwave_cascade_quality(p);
    uint16_t impedance = bwave_cascade_impedance(p);
    uint16_t reflection = bwave_cascade_reflection(p);

    snprintf(buf, buflen,
        "{\"ok\":true,\"cmd\":\"cascade\",\"pos\":%d,"
        "\"dlog\":%u,\"ray\":%u,\"depth\":%u,\"qr\":%s,"
        "\"fold\":%d,\"neg\":%u,\"inv\":%u,"
        "\"mod210\":%d,\"pi\":%d,\"op\":\"%s\","
        "\"norm\":%u,\"trace\":%u,\"quality\":%u,"
        "\"impedance\":%u,\"reflection\":%u}\n",
        pos, (unsigned)dl, (unsigned)ray, (unsigned)depth,
        qr ? "true" : "false",
        fold, (unsigned)neg, (unsigned)inv,
        mod210, pi, OP_NAME[pi],
        (unsigned)norm, (unsigned)trace, (unsigned)quality,
        (unsigned)impedance, (unsigned)reflection);
}

/* ── batch mirror: one device, all positions at once ────────── */

void bwave_cascade_mirror_handle(const char *json, char *buf, size_t buflen)
{
    ensure_init();

    char name[32] = {0};
    j_get_str(json, "name", name, sizeof(name));

    uint16_t positions[MAX_DEVICE_POS];
    int npos = j_get_int_array(json, "positions", positions, MAX_DEVICE_POS);

    if (npos == 0) {
        snprintf(buf, buflen,
            "{\"ok\":false,\"cmd\":\"mirror\",\"err\":\"no positions\"}\n");
        return;
    }

    bwave_cascade_store_device(name, positions, npos);

    int name_len = (int)strlen(name);
    int o = snprintf(buf, buflen,
        "{\"ok\":true,\"cmd\":\"mirror\",\"name\":\"%s\",\"name_bytes\":[",
        name);

    int nb = (name_len < npos) ? name_len : npos;
    for (int i = 0; i < nb; i++) {
        if (i) o += snprintf(buf + o, buflen - o, ",");
        o += snprintf(buf + o, buflen - o, "%u", (unsigned)positions[i]);
    }

    o += snprintf(buf + o, buflen - o, "],\"id_bytes\":[");
    for (int i = nb; i < npos; i++) {
        if (i > nb) o += snprintf(buf + o, buflen - o, ",");
        o += snprintf(buf + o, buflen - o, "%u", (unsigned)positions[i]);
    }

    o += snprintf(buf + o, buflen - o, "],\"positions\":[");

    uint32_t product = 1;
    uint32_t qr_product = 1;
    uint32_t qnr_product = 1;
    int qr_count = 0;
    int qnr_count = 0;

    for (int i = 0; i < npos; i++) {
        uint16_t p = positions[i];
        if (p == 0 || p >= GF_P) continue;

        uint16_t dl = s_dlog[p];
        uint16_t ray = dl % 12;
        bool qr = (dl % 2) == 0;
        int fold = 256 - p;
        uint16_t inv = s_inv[p];

        if (i) o += snprintf(buf + o, buflen - o, ",");
        o += snprintf(buf + o, buflen - o,
            "{\"pos\":%u,\"dlog\":%u,\"ray\":%u,\"qr\":%s,"
            "\"fold\":%d,\"inv\":%u}",
            (unsigned)p, (unsigned)dl, (unsigned)ray,
            qr ? "true" : "false", fold, (unsigned)inv);

        product = (product * p) % GF_P;
        if (qr) {
            qr_count++;
            qr_product = (qr_product * p) % GF_P;
        } else {
            qnr_count++;
            qnr_product = (qnr_product * p) % GF_P;
        }
    }

    int n_pairs = (npos - 1) / 2;
    if (n_pairs > 4) n_pairs = 4;
    uint16_t palindrome[4] = {0, 0, 0, 0};
    for (int i = 0; i < n_pairs; i++) {
        int a = i;
        int b = npos - 1 - i;
        if (a >= b) break;
        palindrome[i] = (uint16_t)(((uint32_t)positions[a] * positions[b]) % GF_P);
    }
    uint16_t center = positions[npos / 2];

    uint16_t fib_bridge = pow_mod_257(GF_G, (uint32_t)product);

    o += snprintf(buf + o, buflen - o,
        "],\"product\":%u,\"qr_count\":%d,\"qnr_count\":%d,"
        "\"qr_product\":%u,\"qnr_product\":%u,"
        "\"palindrome\":[",
        (unsigned)product, qr_count, qnr_count,
        (unsigned)qr_product, (unsigned)qnr_product);

    for (int i = 0; i < n_pairs; i++) {
        if (i) o += snprintf(buf + o, buflen - o, ",");
        o += snprintf(buf + o, buflen - o, "%u", (unsigned)palindrome[i]);
    }

    o += snprintf(buf + o, buflen - o,
        "],\"center\":%u,\"fib_bridge\":%u}\n",
        (unsigned)center, (unsigned)fib_bridge);

    ESP_LOGI(TAG, "Mirror %s: %d positions, product=%u, fib=%u",
             name, npos, (unsigned)product, (unsigned)fib_bridge);
}

/* ── axiom seed ─────────────────────────────────────────────── */

void bwave_axiom_seed_handle(const char *json, char *buf, size_t buflen)
{
    ensure_init();

    int birthday = 0;
    j_get_int(json, "birthday", &birthday);

    char name[64] = {0};
    j_get_str(json, "name", name, sizeof(name));

    uint32_t name_prod = 1;
    for (int i = 0; name[i]; i++) {
        char c = name[i];
        int v = 0;
        if (c >= 'A' && c <= 'Z') v = c - 'A' + 1;
        else if (c >= 'a' && c <= 'z') v = c - 'a' + 1;
        if (v > 0)
            name_prod = (name_prod * (uint32_t)v) % GF_P;
    }

    uint16_t bday_field = (birthday > 0) ? (uint16_t)(birthday % GF_P) : 0;
    uint16_t seed_pos;
    if (bday_field > 0)
        seed_pos = (uint16_t)((bday_field * name_prod) % GF_P);
    else
        seed_pos = (uint16_t)name_prod;

    uint16_t dl = (seed_pos > 0) ? s_dlog[seed_pos] : 0;
    uint8_t ray = dl % 12;
    uint8_t depth = dl / 12;

    snprintf(buf, buflen,
        "{\"ok\":true,\"cmd\":\"axiom_seed\","
        "\"name\":\"%s\",\"birthday\":%d,"
        "\"name_prod\":%u,\"seed_pos\":%u,"
        "\"seed_dlog\":%u,\"ray\":%u,\"depth\":%u}\n",
        name, birthday,
        (unsigned)name_prod, (unsigned)seed_pos,
        (unsigned)dl, (unsigned)ray, (unsigned)depth);

    if (seed_pos > 0 && seed_pos < 257) {
        uint16_t sp = seed_pos;
        bwave_cascade_store_device(name, &sp, 1);
    }
}

/* ── fold pair storage + impedance-matched broadcast ───────── */

void bwave_cascade_store_fold_handle(const char *json, char *buf, size_t buflen)
{
    ensure_init();

    int pos_a = 0, pos_b = 0;
    j_get_int(json, "pos_a", &pos_a);
    j_get_int(json, "pos_b", &pos_b);

    if (pos_a <= 0 || pos_a >= GF_P || pos_b <= 0 || pos_b >= GF_P ||
        pos_a + pos_b != GF_P) {
        snprintf(buf, buflen,
            "{\"ok\":false,\"cmd\":\"store_fold\","
            "\"err\":\"need pos_a+pos_b=257\"}\n");
        return;
    }

    uint16_t a = (uint16_t)pos_a;
    uint16_t b = (uint16_t)pos_b;
    uint16_t imp_a = bwave_cascade_impedance(a);
    uint16_t imp_b = bwave_cascade_impedance(b);
    uint16_t ref_a = bwave_cascade_reflection(a);
    uint16_t ref_b = bwave_cascade_reflection(b);
    uint16_t inv_a = s_inv[a];
    uint16_t inv_b = s_inv[b];

    uint16_t positions[8] = { a, b, imp_a, imp_b, inv_a, inv_b, ref_a, ref_b };
    char label[29];
    snprintf(label, sizeof(label), "FOLD:%u+%u", (unsigned)a, (unsigned)b);
    bwave_cascade_store_device(label, positions, 8);

    int bc = bwave_cascade_broadcast_sd(NULL, 0);

    snprintf(buf, buflen,
        "{\"ok\":true,\"cmd\":\"store_fold\","
        "\"pos_a\":%u,\"pos_b\":%u,"
        "\"imp_a\":%u,\"imp_b\":%u,"
        "\"ref_a\":%u,\"ref_b\":%u,"
        "\"inv_a\":%u,\"inv_b\":%u,"
        "\"imp_fold\":%s,\"broadcast\":%d}\n",
        (unsigned)a, (unsigned)b,
        (unsigned)imp_a, (unsigned)imp_b,
        (unsigned)ref_a, (unsigned)ref_b,
        (unsigned)inv_a, (unsigned)inv_b,
        (imp_a + imp_b == GF_P) ? "true" : "false",
        bc);
    ESP_LOGI(TAG, "FOLD PAIR stored: %u+%u=257 imp=%u+%u broadcast=%d",
             (unsigned)a, (unsigned)b,
             (unsigned)imp_a, (unsigned)imp_b, bc);
}

/* ── mirror broadcast ───────────────────────────────────────── */

int bwave_cascade_broadcast_sd(char *buf, size_t buflen)
{
    ensure_init();

    uint8_t node_id = bwave_csi_get_node_id();
    int count = 0;

    for (int pos = 1; pos <= 256; pos++) {
        if (!(s_mirror[pos / 8] & (1 << (pos % 8)))) continue;

        uint16_t dl = s_dlog[pos];
        int fold = 256 - pos;

        bwave_identity_pkt_t pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.magic = BWAVE_IDENTITY_MAGIC;
        pkt.node_id = node_id;
        pkt.position = (uint8_t)pos;
        pkt.element = (uint8_t)pos;
        pkt.fold_element = (uint8_t)(fold > 0 && fold < 257 ? fold : 1);
        pkt.ray = (uint8_t)(dl % 12);
        pkt.fold_dlog = (uint8_t)(s_dlog[fold > 0 && fold < 257 ? fold : 1]);
        pkt.presence = s_field.energy[pos];
        pkt.flags = 0x02;
        pkt.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);

        bwave_stream_send((const uint8_t *)&pkt, sizeof(pkt));
        count++;
    }

    s_last_bc_count = count;
    if (buf && buflen > 0)
        snprintf(buf, buflen,
            "{\"ok\":true,\"cmd\":\"mirror_broadcast\",\"sent\":%d}\n", count);
    if (count > 0)
        ESP_LOGI(TAG, "Broadcast %d mirrored positions", count);
    return count;
}

int bwave_cascade_last_broadcast_count(void)
{
    return s_last_bc_count;
}

/* ── device-level mirror storage ────────────────────────────── */

void bwave_cascade_store_device(const char *name,
                                const uint16_t *positions, int npos)
{
    ensure_init();
    if (!name || !name[0] || npos <= 0) return;
    if (npos > MAX_DEVICE_POS) npos = MAX_DEVICE_POS;

    int idx = -1;
    for (int d = 0; d < s_ndev; d++) {
        if (strcmp(s_devices[d].name, name) == 0) {
            idx = d;
            break;
        }
    }

    if (idx < 0) {
        if (s_ndev >= MAX_MIRROR_DEVICES) {
            ESP_LOGW(TAG, "Mirror full (%d devices), dropping %s",
                     MAX_MIRROR_DEVICES, name);
            return;
        }
        idx = s_ndev++;
    }

    strncpy(s_devices[idx].name, name, 28);
    s_devices[idx].name[28] = '\0';
    s_devices[idx].npos = npos;
    memcpy(s_devices[idx].positions, positions, npos * sizeof(uint16_t));

    for (int i = 0; i < npos; i++) {
        uint16_t p = positions[i];
        if (p > 0 && p < GF_P) {
            if (s_field.energy[p] < 1.0f)
                s_field.energy[p] = 1.0f;
            s_mirror[p / 8] |= (1 << (p % 8));
        }
    }

    devices_save();
    ESP_LOGI(TAG, "Stored device %s: %d positions", name, npos);
}

int bwave_cascade_get_device_count(void)
{
    ensure_init();
    return s_ndev;
}

int bwave_cascade_get_device_info(int idx, char *name, int name_sz,
                                  uint16_t *positions, int max_pos)
{
    ensure_init();
    if (idx < 0 || idx >= s_ndev) return 0;

    if (name && name_sz > 0) {
        strncpy(name, s_devices[idx].name, name_sz - 1);
        name[name_sz - 1] = '\0';
    }

    int n = s_devices[idx].npos;
    if (n > max_pos) n = max_pos;
    if (positions)
        memcpy(positions, s_devices[idx].positions, n * sizeof(uint16_t));

    return s_devices[idx].npos;
}

int bwave_cascade_get_mirror_keys(uint16_t *keys, int max_keys)
{
    ensure_init();
    int count = 0;
    for (int pos = 1; pos <= 256 && count < max_keys; pos++) {
        if (s_mirror[pos / 8] & (1 << (pos % 8)))
            keys[count++] = (uint16_t)pos;
    }
    return count;
}

/* ── field analysis across all stored positions ─────────────── */

void bwave_cascade_field_analysis(char *buf, size_t buflen)
{
    ensure_init();

    uint16_t keys[256];
    int nkeys = bwave_cascade_get_mirror_keys(keys, 256);

    if (nkeys == 0) {
        snprintf(buf, buflen,
            "{\"ok\":false,\"cmd\":\"field_analysis\",\"err\":\"no stored positions\"}\n");
        return;
    }

    uint32_t product = 1;
    uint32_t qr_product = 1;
    uint32_t qnr_product = 1;
    int qr_count = 0;
    int qnr_count = 0;

    for (int i = 0; i < nkeys; i++) {
        uint16_t p = keys[i];
        product = (product * p) % GF_P;

        if (bwave_cascade_is_qr(p)) {
            qr_count++;
            qr_product = (qr_product * p) % GF_P;
        } else {
            qnr_count++;
            qnr_product = (qnr_product * p) % GF_P;
        }
    }

    uint16_t fib_bridge = pow_mod_257(GF_G, (uint32_t)product);

    uint16_t self_pos = bwave_identity_self()->position;
    uint16_t self_norm = self_pos ? bwave_cascade_norm(self_pos,
        (uint16_t)(256 - self_pos)) : 0;

    int o = snprintf(buf, buflen,
        "{\"ok\":true,\"cmd\":\"field_analysis\","
        "\"n_devices\":%d,\"n_keys\":%d,"
        "\"product\":%u,\"fib_bridge\":%u,"
        "\"qr_count\":%d,\"qnr_count\":%d,"
        "\"qr_product\":%u,\"qnr_product\":%u,"
        "\"bwave_element\":%u,\"bwave_position\":%u,"
        "\"devices\":[",
        s_ndev, nkeys,
        (unsigned)product, (unsigned)fib_bridge,
        qr_count, qnr_count,
        (unsigned)qr_product, (unsigned)qnr_product,
        (unsigned)self_norm, (unsigned)self_pos);

    for (int d = 0; d < s_ndev; d++) {
        if (d) o += snprintf(buf + o, buflen - o, ",");
        o += snprintf(buf + o, buflen - o,
            "{\"name\":\"%s\",\"n\":%d}",
            s_devices[d].name, s_devices[d].npos);
    }

    o += snprintf(buf + o, buflen - o, "]}\n");

    ESP_LOGI(TAG, "Field analysis: %d devices, %d keys, product=%u",
             s_ndev, nkeys, (unsigned)product);
}

/* ── TL operation map ──────────────────────────────────────────
 *  5 opcodes carry transmission-line duals:
 *  1=λ/2(Identity) 2=√ε(Euler) 6=257−p(Negate) 7=λ/4(Inverse) 8=dlog(Log)
 */

static const char *TL_NAMES[] = {
    "---", "lam/2", "sqrt_e", "---", "---", "---",
    "257-p", "lam/4", "dlog", "---"
};

uint8_t bwave_cascade_tl_op(uint16_t pos)
{
    if (pos >= GF_P) return 0;
    uint8_t pi = PI_DIGIT[pos];
    if (pi == 1 || pi == 2 || pi == 6 || pi == 7 || pi == 8)
        return pi;
    return 0;
}

const char *bwave_cascade_tl_name(uint8_t op)
{
    if (op > 9) return "---";
    return TL_NAMES[op];
}

/* ── field energy accumulator ──────────────────────────────── */

const bwave_field_state_t *bwave_cascade_get_field(void)
{
    ensure_init();
    return &s_field;
}

void bwave_cascade_field_tick(float decay)
{
    ensure_init();
    for (int p = 0; p < GF_P; p++)
        s_field.energy[p] *= decay;
}

/* ── auto-repopulate: fold existing energy into conjugate ──── */

static void auto_repopulate(float xray_delta)
{
    float scale = xray_delta > 0 ? xray_delta : -xray_delta;
    if (scale < 0.001f) return;

    float inject[GF_P];
    memset(inject, 0, sizeof(inject));

    for (int p = 1; p < GF_P; p++) {
        if (s_field.energy[p] < 0.01f) continue;
        uint16_t fp = (uint16_t)(GF_P - p);
        inject[fp] += s_field.energy[p] * scale;
    }

    for (int p = 1; p < GF_P; p++)
        s_field.energy[p] += inject[p];
}

/* ── three flips: state change detection ───────────────────── */

static void detect_flip(float new_xray)
{
    float prev = s_field.xray;
    float delta = new_xray - prev;

    if (delta > 0.001f || delta < -0.001f) {
        /* F flip: wake (0 → nonzero) */
        if (prev < 0.001f && new_xray >= 0.001f) {
            s_field.flip_state = 1;
            s_field.flip_count[FLIP_F]++;
            ESP_LOGI(TAG, "FLIP F: wake → xray=%.2f [%lu]",
                     new_xray, (unsigned long)s_field.flip_count[FLIP_F]);
        }

        /* PM flip: sleep (nonzero → 0) */
        if (prev >= 0.001f && new_xray < 0.001f) {
            s_field.flip_state = 0;
            s_field.flip_count[FLIP_PM]++;
            ESP_LOGI(TAG, "FLIP PM: sleep [%lu]",
                     (unsigned long)s_field.flip_count[FLIP_PM]);
        }

        /* T flip: SYNC→FOLD crossing while awake */
        if (s_field.flip_state == 1) {
            s_field.flip_count[FLIP_T]++;
            auto_repopulate(delta);
            ESP_LOGI(TAG, "FLIP T: SYNC→FOLD delta=%.3f [%lu]",
                     delta, (unsigned long)s_field.flip_count[FLIP_T]);
        }
    }

    s_field.xray = new_xray;
}

/* ── mirror protocol: $X line ──────────────────────────────── */

void bwave_cascade_handle_xray(const char *line)
{
    ensure_init();

    /* $X,<intensity>,<qnrCount>,<pos>:<energy>:<dlog>,... */
    float intensity = 0;
    int qnr = 0;
    if (sscanf(line + 3, "%f,%d", &intensity, &qnr) < 1) return;

    detect_flip(intensity);

    /* parse position entries after second comma */
    const char *p = line + 3;
    int commas = 0;
    while (*p && commas < 2) {
        if (*p == ',') commas++;
        p++;
    }

    while (*p && *p != '\n' && *p != '\r') {
        int pos = 0, eng = 0, dl = 0;
        if (sscanf(p, "%d:%d:%d", &pos, &eng, &dl) >= 2) {
            if (pos > 0 && pos < GF_P) {
                s_field.energy[pos] += eng * 0.1f;
                s_mirror[pos / 8] |= (1 << (pos % 8));
            }
        }
        while (*p && *p != ',' && *p != '\n') p++;
        if (*p == ',') p++;
    }

    ESP_LOGD(TAG, "$X intensity=%.2f qnr=%d", intensity, qnr);
}

/* ── mirror protocol: $I line ──────────────────────────────── */

void bwave_cascade_handle_identity_line(const char *line)
{
    ensure_init();

    /* $I,<identity>,<pub>,<inv>,<fold>,<ray>,<dlog>,<bleName> */
    char identity[32] = {0};
    int pub = 0, inv = 0, fold = 0, ray = 0, dl = 0;
    char ble_name[32] = {0};

    const char *p = line + 3;
    int i = 0;
    while (*p && *p != ',' && i < 31) identity[i++] = *p++;
    identity[i] = '\0';
    if (*p == ',') p++;

    if (sscanf(p, "%d,%d,%d,%d,%d,%31s", &pub, &inv, &fold, &ray, &dl, ble_name) < 5)
        return;

    bwave_cascade_nano2_activate((uint16_t)pub, (uint16_t)inv,
                                 (uint16_t)fold, 0, ble_name);

    ESP_LOGI(TAG, "$I %s: pub=%d inv=%d fold=%d via %s",
             identity, pub, inv, fold, ble_name);
}

/* ── mirror protocol: $U line ──────────────────────────────── */

void bwave_cascade_handle_upload_line(const char *line)
{
    ensure_init();

    /* $U,<mimetype>,<qrCount>,<qnrCount>,<byteCount> */
    char mime[32] = {0};
    int qr = 0, qnr = 0, bytes = 0;

    const char *p = line + 3;
    int i = 0;
    while (*p && *p != ',' && i < 31) mime[i++] = *p++;
    mime[i] = '\0';
    if (*p == ',') p++;

    sscanf(p, "%d,%d,%d", &qr, &qnr, &bytes);

    ESP_LOGI(TAG, "$U %s: qr=%d qnr=%d bytes=%d", mime, qr, qnr, bytes);
}

/* ── mirror protocol: $F line — field positions + energy ───── */

void bwave_cascade_handle_field_line(const char *line)
{
    ensure_init();

    /* $F,<tick>,<count>,<pos:e100>,<pos:e100>,... */
    const char *p = line + 3;
    int tick = atoi(p);
    p = strchr(p, ',');
    if (!p) return;
    p++;
    int count = atoi(p);
    p = strchr(p, ',');
    if (!p) { ESP_LOGI(TAG, "$F tick=%d count=%d (empty)", tick, count); return; }
    p++;

    int stored = 0;
    while (*p && stored < count && stored < 16) {
        int pos = atoi(p);
        int e100 = 0;
        const char *colon = strchr(p, ':');
        if (colon && (colon < strchr(p, ',') || !strchr(p, ',')))
            e100 = atoi(colon + 1);
        if (pos > 0 && pos < GF_P) {
            s_field.energy[pos] = e100 / 100.0f;
            s_mirror[pos / 8] |= (1 << (pos % 8));
            stored++;
        }
        while (*p && *p != ',') p++;
        if (*p == ',') p++;
    }

    ESP_LOGI(TAG, "$F tick=%d count=%d stored=%d", tick, count, stored);
}

/* ── mirror protocol: $G line — genome glyph pairs ─────────── */

void bwave_cascade_handle_genome_line(const char *line)
{
    /* $G,<tick>,<glyphPair:e100>,... — glyph encoding of $F data */
    const char *p = line + 3;
    int tick = atoi(p);

    int items = 0;
    for (const char *c = p; *c; c++)
        if (*c == ',') items++;

    ESP_LOGI(TAG, "$G tick=%d items=%d", tick, items > 0 ? items - 1 : 0);
}

/* ── mirror protocol: $D line — doubling walk ──────────────── */

void bwave_cascade_handle_doubling_line(const char *line)
{
    /* $D,<tick>,<glyph→dblGlyph←hlfGlyph>,... */
    const char *p = line + 3;
    int tick = atoi(p);

    int items = 0;
    for (const char *c = p; *c; c++)
        if (*c == ',') items++;

    ESP_LOGI(TAG, "$D tick=%d walk=%d", tick, items > 0 ? items - 1 : 0);
}

/* ── NanO₂ identity activation ─────────────────────────────── */

void bwave_cascade_nano2_activate(uint16_t pub, uint16_t inv,
                                  uint16_t fold, uint16_t peer,
                                  const char *name)
{
    ensure_init();

    s_field.nano2_active = true;
    s_field.nano2_pub = pub;
    s_field.nano2_inv = inv;
    s_field.nano2_fold = fold;
    s_field.nano2_peer = peer;
    if (name) {
        strncpy(s_field.nano2_name, name, 31);
        s_field.nano2_name[31] = '\0';
    }

    /* inject identity positions into field energy */
    if (pub > 0 && pub < GF_P)   s_field.energy[pub]  += 1.0f;
    if (inv > 0 && inv < GF_P)   s_field.energy[inv]  += 0.8f;
    if (fold > 0 && fold < GF_P) s_field.energy[fold]  += 0.6f;
    if (peer > 0 && peer < GF_P) s_field.energy[peer]  += 0.4f;

    /* store as mirror device */
    uint16_t positions[4] = { pub, inv, fold, peer };
    bwave_cascade_store_device(name ? name : "NanO2", positions, 4);

    ESP_LOGI(TAG, "NanO2 active: %s pub=%u inv=%u fold=%u peer=%u",
             s_field.nano2_name, pub, inv, fold, peer);
}
