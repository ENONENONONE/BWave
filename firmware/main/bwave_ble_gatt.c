#include "bwave_ble_gatt.h"
#include "bwave_ble.h"
#include "bwave_cascade.h"
#include "bwave_cmd.h"
#include <string.h>
#include "esp_log.h"
#include "host/ble_hs.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "bwave_gatt";

/* ── Cascade GATT Service ──────────────────────────────────────
 *  Service  C5110001-0257-0003-8000-00805F9B34FB
 *  Chars:
 *    0002  Field Query   (W: pos uint16 → R: 22-byte decomposition)
 *    0003  Mirror State  (R: active positions + energy;
 *                         W: [count][pos_lo,pos_hi,e_lo,e_hi]... inject energy)
 *    0004  Device List   (R: registered devices;
 *                         W: [name(12)][npos][pos_lo,pos_hi]... store device)
 *    0005  Scan Control  (R: BLE scan results;
 *                         W: any byte → trigger BLE scan)
 *    0006  Command       (W: JSON command → R: JSON response, max 512 bytes)
 */

static const ble_uuid128_t svc_uuid = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x03, 0x00, 0x57, 0x02, 0x01, 0x00, 0x11, 0xc5);

static const ble_uuid128_t chr_query_uuid = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x03, 0x00, 0x57, 0x02, 0x02, 0x00, 0x11, 0xc5);

static const ble_uuid128_t chr_mirror_uuid = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x03, 0x00, 0x57, 0x02, 0x03, 0x00, 0x11, 0xc5);

static const ble_uuid128_t chr_devices_uuid = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x03, 0x00, 0x57, 0x02, 0x04, 0x00, 0x11, 0xc5);

static const ble_uuid128_t chr_scan_uuid = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x03, 0x00, 0x57, 0x02, 0x05, 0x00, 0x11, 0xc5);

static const ble_uuid128_t chr_cmd_uuid = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x03, 0x00, 0x57, 0x02, 0x06, 0x00, 0x11, 0xc5);

static uint16_t s_query_pos = 1;
static char s_cmd_resp[2048];
static int  s_cmd_resp_len = 0;

/* ── 0002 Field Query: write position, read decomposition ───── */

struct __attribute__((packed)) field_query_rsp {
    uint16_t pos;
    uint16_t dlog;
    uint8_t  ray;
    uint8_t  qr;
    uint16_t fold;
    uint16_t inv;
    uint8_t  tl_op;
    uint8_t  reserved;
    int16_t  energy_x100;
    uint16_t neg;
    uint16_t quality;
    uint16_t impedance;
    uint16_t reflection;
};

static int query_access(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len >= 2) {
            uint8_t b[2];
            ble_hs_mbuf_to_flat(ctxt->om, b, 2, NULL);
            s_query_pos = b[0] | ((uint16_t)b[1] << 8);
        } else if (len >= 1) {
            uint8_t b;
            ble_hs_mbuf_to_flat(ctxt->om, &b, 1, NULL);
            s_query_pos = b;
        }
        if (s_query_pos == 0 || s_query_pos >= GF_P)
            s_query_pos = 1;
        return 0;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint16_t p = s_query_pos;
        const bwave_field_state_t *fs = bwave_cascade_get_field();
        uint16_t dl = bwave_cascade_dlog(p);

        struct field_query_rsp r;
        r.pos        = p;
        r.dlog       = dl;
        r.ray        = dl % 12;
        r.qr         = (dl % 2 == 0) ? 1 : 0;
        r.fold       = (uint16_t)(256 - p);
        r.inv        = bwave_cascade_inv(p);
        r.tl_op      = bwave_cascade_tl_op(p);
        r.reserved   = 0;
        r.energy_x100 = (int16_t)(fs->energy[p] * 100.0f);
        r.neg        = bwave_cascade_neg(p);
        r.quality    = bwave_cascade_quality(p);
        r.impedance  = bwave_cascade_impedance(p);
        r.reflection = bwave_cascade_reflection(p);

        os_mbuf_append(ctxt->om, &r, sizeof(r));
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

/* ── 0003 Mirror State ────────────────────────────────────────
 *  READ:  [count] [pos_lo,pos_hi,energy_x100_lo,energy_x100_hi]...
 *  WRITE: [count] [pos_lo,pos_hi,energy_x100_lo,energy_x100_hi]...
 *         Injects energy at the written positions.
 */

static int mirror_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len < 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        uint8_t buf[512];
        uint16_t rlen = len > sizeof(buf) ? sizeof(buf) : len;
        ble_hs_mbuf_to_flat(ctxt->om, buf, rlen, NULL);

        uint8_t count = buf[0];
        int off = 1;
        bwave_field_state_t *fs =
            (bwave_field_state_t *)bwave_cascade_get_field();
        int injected = 0;
        for (int i = 0; i < count && off + 4 <= rlen; i++) {
            uint16_t pos;
            int16_t e;
            memcpy(&pos, buf + off, 2); off += 2;
            memcpy(&e, buf + off, 2);   off += 2;
            if (pos > 0 && pos < GF_P) {
                float ef = e / 100.0f;
                if (ef > fs->energy[pos])
                    fs->energy[pos] = ef;
                injected++;
            }
        }
        ESP_LOGI(TAG, "GATT mirror write: %d positions injected", injected);
        return 0;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        const bwave_field_state_t *fs = bwave_cascade_get_field();
        uint8_t buf[512];
        int off = 1;
        uint8_t count = 0;

        for (int p = 1; p < GF_P && off + 4 <= (int)sizeof(buf); p++) {
            if (fs->energy[p] < 0.01f) continue;
            uint16_t pos = (uint16_t)p;
            int16_t e = (int16_t)(fs->energy[p] * 100.0f);
            memcpy(buf + off, &pos, 2); off += 2;
            memcpy(buf + off, &e, 2);   off += 2;
            count++;
        }
        buf[0] = count;

        os_mbuf_append(ctxt->om, buf, off);
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

/* ── 0004 Device List ─────────────────────────────────────────
 *  READ:  [ndev] [name(12),npos(1),max_energy(2)]...
 *  WRITE: [name(12)] [npos(1)] [pos_lo,pos_hi]...
 *         Stores a named device with its field positions.
 */

static int devices_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len < 14) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        uint8_t buf[256];
        uint16_t rlen = len > sizeof(buf) ? sizeof(buf) : len;
        ble_hs_mbuf_to_flat(ctxt->om, buf, rlen, NULL);

        char name[13] = {0};
        memcpy(name, buf, 12);
        name[12] = '\0';
        uint8_t npos = buf[12];
        if (npos > MAX_DEVICE_POS) npos = MAX_DEVICE_POS;

        uint16_t positions[MAX_DEVICE_POS];
        int off = 13;
        int actual = 0;
        for (int i = 0; i < npos && off + 2 <= rlen; i++) {
            memcpy(&positions[i], buf + off, 2);
            off += 2;
            actual++;
        }

        if (actual > 0 && name[0]) {
            bwave_cascade_store_device(name, positions, actual);
            ESP_LOGI(TAG, "GATT device store: %s (%d pos)", name, actual);
        }
        return 0;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        const bwave_field_state_t *fs = bwave_cascade_get_field();
        uint8_t buf[512];
        int off = 0;

        int ndev = bwave_cascade_get_device_count();
        buf[off++] = (uint8_t)ndev;

        for (int i = 0; i < ndev && off + 15 <= (int)sizeof(buf); i++) {
            char name[13] = {0};
            uint16_t dpos[MAX_DEVICE_POS];
            int npos = bwave_cascade_get_device_info(i, name, sizeof(name),
                                                      dpos, MAX_DEVICE_POS);
            float emax = 0.0f;
            for (int j = 0; j < npos; j++) {
                if (dpos[j] > 0 && dpos[j] < GF_P && fs->energy[dpos[j]] > emax)
                    emax = fs->energy[dpos[j]];
            }

            memset(buf + off, 0, 12);
            int nlen = (int)strlen(name);
            if (nlen > 12) nlen = 12;
            memcpy(buf + off, name, nlen);
            off += 12;
            buf[off++] = (uint8_t)(npos > 255 ? 255 : npos);
            int16_t ex = (int16_t)(emax * 100.0f);
            memcpy(buf + off, &ex, 2);
            off += 2;
        }

        os_mbuf_append(ctxt->om, buf, off);
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

/* ── 0005 Scan Control ────────────────────────────────────────
 *  READ:  BLE scan results [count][name(12),rssi,position,mirrored]...
 *  WRITE: any byte → trigger BLE scan (5s)
 */

static int scan_access(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        bwave_ble_scan_start();
        ESP_LOGI(TAG, "GATT scan trigger");
        return 0;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        const bwave_ble_device_t *devs = bwave_ble_get_devices();
        int ndev = bwave_ble_get_device_count();
        uint8_t buf[512];
        int off = 0;

        buf[off++] = (uint8_t)(ndev > 32 ? 32 : ndev);

        for (int i = 0; i < ndev && i < 32 && off + 15 <= (int)sizeof(buf); i++) {
            memset(buf + off, 0, 12);
            int nlen = (int)strlen(devs[i].name);
            if (nlen > 12) nlen = 12;
            memcpy(buf + off, devs[i].name, nlen);
            off += 12;
            buf[off++] = (uint8_t)(int8_t)devs[i].rssi;
            buf[off++] = devs[i].position;
            buf[off++] = devs[i].mirrored;
        }

        os_mbuf_append(ctxt->om, buf, off);
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

/* ── 0006 Command: write JSON → read JSON response ───────────
 *  WRITE: UTF-8 JSON command (same format as serial)
 *  READ:  last JSON response (up to 512 bytes)
 *  Routes through bwave_cmd_handle_line() — full command set.
 */

static int cmd_access(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len == 0) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

        char cmd_buf[256];
        uint16_t rlen = len > sizeof(cmd_buf) - 1 ? sizeof(cmd_buf) - 1 : len;
        ble_hs_mbuf_to_flat(ctxt->om, cmd_buf, rlen, NULL);
        cmd_buf[rlen] = '\0';

        while (rlen > 0 && (cmd_buf[rlen-1] == '\n' || cmd_buf[rlen-1] == '\r'))
            cmd_buf[--rlen] = '\0';

        s_cmd_resp[0] = '\0';
        s_cmd_resp_len = 0;

        bwave_cmd_handle_line(cmd_buf, s_cmd_resp, sizeof(s_cmd_resp));
        s_cmd_resp_len = (int)strlen(s_cmd_resp);

        ESP_LOGI(TAG, "GATT cmd: %s → %d bytes", cmd_buf, s_cmd_resp_len);
        return 0;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        if (s_cmd_resp_len > 0) {
            os_mbuf_append(ctxt->om, s_cmd_resp, s_cmd_resp_len);
        } else {
            const char *empty = "{\"ok\":true,\"cmd\":\"nop\"}\n";
            os_mbuf_append(ctxt->om, empty, strlen(empty));
        }
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

/* ── Service definition ────────────────────────────────────── */

static const struct ble_gatt_svc_def s_cascade_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &chr_query_uuid.u,
                .access_cb = query_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &chr_mirror_uuid.u,
                .access_cb = mirror_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &chr_devices_uuid.u,
                .access_cb = devices_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &chr_scan_uuid.u,
                .access_cb = scan_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &chr_cmd_uuid.u,
                .access_cb = cmd_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            { 0 },
        },
    },
    { 0 },
};

void bwave_ble_gatt_init(void)
{
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_cascade_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT count failed: %d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(s_cascade_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT add failed: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "Cascade GATT service registered (6 chars, all R+W)");
}
