#include "bwave_cmd.h"
#include "bwave_csi.h"
#include "bwave_dsp.h"
#include "bwave_identity.h"
#include "bwave_ble.h"
#include "bwave_cascade.h"
#include "bwave_config.h"
#include "bwave_slip.h"
#include "bwave_lcd.h"
#include "bwave_sd.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "bwave_cmd";

static const char *GENOME_SYM[16] = {
    "\xCE\xA6",        /* 0: Φ */
    "\xE2\x88\x82",    /* 1: ∂ */
    "\xCF\x86",        /* 2: φ */
    "\xE2\x88\x9E",    /* 3: ∞ */
    "\xE2\x8A\x9F",    /* 4: ⊟ */
    "\xE2\x8A\x97",    /* 5: ⊗ */
    "\xE2\x99\xAF",    /* 6: ♯ */
    "\xE2\x85\x97",    /* 7: ⅗ */
    "\xCE\xB5",        /* 8: ε */
    "\xCE\xB4",        /* 9: δ */
    "\xCE\xA9",        /* A: Ω */
    "\xCE\x93",        /* B: Γ */
    "Q",                /* C: Q */
    "\xCE\xA3",        /* D: Σ */
    "\xCF\x88",        /* E: ψ */
    "\xF0\x9D\x9F\x99" /* F: 𝟙 */
};

static int write_glyph(char *buf, int max, uint16_t pos)
{
    if (pos > 255) {
        uint8_t top = (pos >> 8) & 0xF;
        uint8_t hi  = (pos >> 4) & 0xF;
        uint8_t lo  = pos & 0xF;
        return snprintf(buf, max, "%s%s%s", GENOME_SYM[top], GENOME_SYM[hi], GENOME_SYM[lo]);
    }
    uint8_t hi = (pos >> 4) & 0xF;
    uint8_t lo = pos & 0xF;
    return snprintf(buf, max, "%s%s", GENOME_SYM[hi], GENOME_SYM[lo]);
}

static void usb_raw_send(const char *str)
{
    usb_serial_jtag_write_bytes(str, strlen(str), pdMS_TO_TICKS(100));
}

static void handle_tunnel_line(const char *line)
{
    const char *p = line + 3;

    if (strncmp(p, "OPEN", 4) == 0) {
        usb_raw_send("!Q,ACK,OPEN\n");
        ESP_LOGI(TAG, "Tunnel OPEN — x55");
        return;
    }

    if (strncmp(p, "CLOSE", 5) == 0) {
        usb_raw_send("!Q,ACK,CLOSE\n");
        ESP_LOGI(TAG, "Tunnel CLOSE");
        return;
    }

    if (strncmp(p, "DATA", 4) == 0) {
        p += 5;
        int seq = atoi(p);

        p = strchr(p, ',');
        if (!p) { usb_raw_send("!Q,NACK,0,parse\n"); return; }
        p++;
        int count = atoi(p);

        p = strchr(p, ',');
        if (!p) {
            char nack[64];
            snprintf(nack, sizeof(nack), "!Q,NACK,%d,empty\n", seq);
            usb_raw_send(nack);
            return;
        }
        p++;

        char ack[64];
        snprintf(ack, sizeof(ack), "!Q,ACK,%d\n", seq);
        usb_raw_send(ack);

        char data_buf[256];
        char echo_buf[512];
        int dpos = snprintf(data_buf, sizeof(data_buf), "!Q,DATA,%d", seq);
        int epos = snprintf(echo_buf, sizeof(echo_buf), "!Q,ECHO");

        int parsed = 0;
        while (*p && parsed < count && parsed < 16) {
            int pos = atoi(p);
            if (pos > 0 && pos < GF_P) {
                uint16_t w = ((uint32_t)pos * 55) % GF_P;
                if (dpos < (int)sizeof(data_buf) - 8)
                    dpos += snprintf(data_buf + dpos, sizeof(data_buf) - dpos,
                                     ",%d", w);
                if (epos < (int)sizeof(echo_buf) - 16) {
                    epos += snprintf(echo_buf + epos, sizeof(echo_buf) - epos, ",");
                    epos += write_glyph(echo_buf + epos, sizeof(echo_buf) - epos,
                                        (uint16_t)pos);
                    epos += snprintf(echo_buf + epos, sizeof(echo_buf) - epos, ">");
                    epos += write_glyph(echo_buf + epos, sizeof(echo_buf) - epos, w);
                }
                parsed++;
            }
            while (*p && *p != ',') p++;
            if (*p == ',') p++;
        }

        dpos += snprintf(data_buf + dpos, sizeof(data_buf) - dpos, "\n");
        usb_raw_send(data_buf);

        epos += snprintf(echo_buf + epos, sizeof(echo_buf) - epos, "\n");
        usb_raw_send(echo_buf);
        return;
    }

    usb_raw_send("!Q,NACK,0,unknown\n");
}

/* "key": number anywhere in a flat JSON object; false if absent */
static bool json_num(const char *json, const char *key, float *out)
{
    char pat[24];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (!p) return false;
    p = strchr(p + strlen(pat), ':');
    if (!p) return false;
    char *end;
    float v = strtof(p + 1, &end);
    if (end == p + 1) return false;
    *out = v;
    return true;
}

static int res_peak_json(char *buf, size_t len, const char *name,
                         const bwave_res_peak_t *pk)
{
    return snprintf(buf, len,
        "\"%s\":{\"valid\":%s,\"f0\":%.4f,\"bpm\":%.2f,\"f1\":%.4f,"
        "\"f2\":%.4f,\"q\":%.2f,\"snr\":%.1f,\"res_limited\":%s}",
        name, pk->valid ? "true" : "false", pk->f0_hz, pk->f0_hz * 60.0f,
        pk->f1_hz, pk->f2_hz, pk->q, pk->snr,
        pk->res_limited ? "true" : "false");
}

static int filter_json(char *buf, size_t len, const bwave_dsp_filter_t *f)
{
    float b1, b2, h1, h2;
    bwave_res_band_edges(f->br_f0, f->br_q, f->br_stages, &b1, &b2);
    bwave_res_band_edges(f->hr_f0, f->hr_q, f->hr_stages, &h1, &h2);
    return snprintf(buf, len,
        "\"filter\":{\"br\":{\"f0\":%.4f,\"q\":%.3f,\"stages\":%u,"
        "\"f1\":%.4f,\"f2\":%.4f},"
        "\"hr\":{\"f0\":%.4f,\"q\":%.3f,\"stages\":%u,"
        "\"f1\":%.4f,\"f2\":%.4f}}",
        f->br_f0, f->br_q, f->br_stages, b1, b2,
        f->hr_f0, f->hr_q, f->hr_stages, h1, h2);
}

static void handle_cmd(const char *json, char *buf, size_t buflen)
{
    char cmd[32] = {0};
    const char *p = strstr(json, "\"cmd\"");
    if (!p) return;
    p = strchr(p + 5, '"');
    if (!p) return;
    p++;
    int i = 0;
    while (*p && *p != '"' && i < 31) cmd[i++] = *p++;
    cmd[i] = 0;

    if (strcmp(cmd, "bio") == 0) {
        bwave_csi_snapshot_t snap;
        bwave_csi_get_snapshot(&snap);
        bwave_vitals_pkt_t pkt;
        bool has_vitals = bwave_dsp_get_vitals(&pkt);

        float delta, theta, alpha;
        bwave_dsp_get_brainwave(&delta, &theta, &alpha);

        int pos = snprintf(buf, buflen,
            "{\"ok\":true,\"cmd\":\"bio\",\"node\":%u,"
            "\"rssi\":%d,\"ch\":%d,\"cb\":%lu,\"iq_len\":%d",
            bwave_csi_get_node_id(),
            snap.rssi, snap.channel,
            (unsigned long)snap.cb_count, snap.iq_len);

        if (has_vitals) {
            pos += snprintf(buf + pos, buflen - pos,
                ",\"br\":%.2f,\"hr\":%.2f,\"presence\":%.4f,"
                "\"motion\":%.4f,\"temp\":%d,\"n_persons\":%u,"
                "\"fall\":%s",
                pkt.breathing_rate / 100.0f,
                pkt.heartrate / 10000.0f,
                pkt.presence_score,
                pkt.motion_energy,
                pkt.die_temp_c, pkt.n_persons,
                (pkt.flags & 2) ? "true" : "false");
        }

        pos += snprintf(buf + pos, buflen - pos,
            ",\"delta\":%.6f,\"theta\":%.6f,\"alpha\":%.6f",
            delta, theta, alpha);

        pos += snprintf(buf + pos, buflen - pos, ",\"iq\":[");
        int n_sc = snap.iq_len / 2;
        if (n_sc > 64) n_sc = 64;
        for (int j = 0; j < n_sc; j++) {
            if (j) buf[pos++] = ',';
            pos += snprintf(buf + pos, buflen - pos,
                "[%d,%d]", snap.iq[j * 2], snap.iq[j * 2 + 1]);
        }
        pos += snprintf(buf + pos, buflen - pos, "]}\n");

    } else if (strcmp(cmd, "vitals") == 0) {
        bwave_vitals_pkt_t pkt;
        bool ok = bwave_dsp_get_vitals(&pkt);
        if (ok) {
            snprintf(buf, buflen,
                "{\"ok\":true,\"cmd\":\"vitals\",\"node\":%u,\"flags\":%u,"
                "\"br\":%.2f,\"hr\":%.2f,\"rssi\":%d,\"n_persons\":%u,"
                "\"motion\":%.4f,\"presence\":%.4f,\"temp\":%d,\"ts\":%lu}\n",
                pkt.node_id, pkt.flags,
                pkt.breathing_rate / 100.0f, pkt.heartrate / 10000.0f,
                pkt.rssi, pkt.n_persons,
                pkt.motion_energy, pkt.presence_score,
                pkt.die_temp_c, (unsigned long)pkt.timestamp_ms);
        } else {
            snprintf(buf, buflen,
                "{\"ok\":false,\"cmd\":\"vitals\",\"err\":\"no data yet\"}\n");
        }

    } else if (strcmp(cmd, "brainwave") == 0) {
        float delta, theta, alpha;
        bwave_dsp_get_brainwave(&delta, &theta, &alpha);
        bwave_csi_snapshot_t snap;
        bwave_csi_get_snapshot(&snap);

        int pos = snprintf(buf, buflen,
            "{\"ok\":true,\"cmd\":\"brainwave\","
            "\"delta\":%.6f,\"theta\":%.6f,\"alpha\":%.6f,"
            "\"rssi\":%d,",
            delta, theta, alpha, snap.rssi);

        const bwave_field_state_t *fs = bwave_cascade_get_field();
        uint16_t fp = (uint16_t)((unsigned)(snap.rssi < 0 ? -snap.rssi : snap.rssi) % GF_P);
        if (fp == 0) fp = 1;
        uint16_t dl = bwave_cascade_dlog(fp);
        pos += snprintf(buf + pos, buflen - pos,
            "\"field\":{\"pos\":%u,\"dlog\":%u,\"ray\":%u,\"qr\":%s,"
            "\"energy\":%.2f,\"xray\":%.2f,\"flip\":%lu,\"nano2\":%s}}\n",
            fp, dl, bwave_cascade_ray(fp),
            bwave_cascade_is_qr(fp) ? "true" : "false",
            fs->energy[fp], fs->xray,
            fs->flip_count[FLIP_F] + fs->flip_count[FLIP_T] + fs->flip_count[FLIP_PM],
            fs->nano2_active ? "true" : "false");

    } else if (strcmp(cmd, "csi_read") == 0) {
        bwave_csi_snapshot_t snap;
        bwave_csi_get_snapshot(&snap);

        int pos = snprintf(buf, buflen,
            "{\"ok\":true,\"cmd\":\"csi_read\",\"rssi\":%d,\"ch\":%d,"
            "\"cb\":%lu,\"iq_len\":%d,\"amp\":[",
            snap.rssi, snap.channel,
            (unsigned long)snap.cb_count, snap.iq_len);

        int n_sc = snap.iq_len / 2;
        if (n_sc > 64) n_sc = 64;
        for (int j = 0; j < n_sc; j++) {
            float im = snap.iq[j * 2];
            float re = snap.iq[j * 2 + 1];
            int amp = (int)sqrtf(im * im + re * re);
            if (j) buf[pos++] = ',';
            pos += snprintf(buf + pos, buflen - pos, "%d", amp);
        }
        pos += snprintf(buf + pos, buflen - pos, "]}\n");

    } else if (strcmp(cmd, "identity") == 0) {
        bwave_identity_get_json(buf, buflen);

    } else if (strcmp(cmd, "identities") == 0) {
        bwave_identity_list_sd(buf, buflen);

    } else if (strcmp(cmd, "ble_scan") == 0) {
        bwave_ble_scan_start();
        snprintf(buf, buflen,
            "{\"ok\":true,\"cmd\":\"ble_scan\",\"msg\":\"scanning 5s\"}\n");

    } else if (strcmp(cmd, "ble_macs") == 0) {
        bwave_ble_get_macs_json(buf, buflen);

    } else if (strcmp(cmd, "cascade") == 0) {
        bwave_cascade_handle(json, buf, buflen);

    } else if (strcmp(cmd, "axiom_seed") == 0) {
        bwave_axiom_seed_handle(json, buf, buflen);

    } else if (strcmp(cmd, "declare") == 0) {
        bwave_declared_t d = { .active = true, .layers_measured = 0 };
        const char *pp;

        pp = strstr(json, "\"name\"");
        if (pp) {
            pp = strchr(pp + 6, '"'); if (pp) { pp++;
            int n = 0; while (*pp && *pp != '"' && n < 31) d.name[n++] = *pp++;
            d.name[n] = 0; }
        }

        pp = strstr(json, "\"pos\"");
        if (pp) { pp = strchr(pp + 5, ':'); if (pp) d.position = (uint8_t)atoi(pp + 1); }

        pp = strstr(json, "\"inv\"");
        if (pp) { pp = strchr(pp + 5, ':'); if (pp) d.inverse = (uint8_t)atoi(pp + 1); }

        pp = strstr(json, "\"ray\"");
        if (pp) { pp = strchr(pp + 5, ':'); if (pp) d.ray = (uint8_t)atoi(pp + 1); }

        pp = strstr(json, "\"dlog\"");
        if (pp) { pp = strchr(pp + 6, ':'); if (pp) d.dlog = (uint8_t)atoi(pp + 1); }

        pp = strstr(json, "\"element\"");
        if (pp) { pp = strchr(pp + 9, ':'); if (pp) d.element = (uint8_t)atoi(pp + 1); }

        pp = strstr(json, "\"layers\"");
        if (pp) { pp = strchr(pp + 8, ':'); if (pp) d.layers_measured = (uint8_t)atoi(pp + 1); }

        pp = strstr(json, "\"hash\"");
        if (pp) {
            pp = strchr(pp + 6, '"'); if (pp) { pp++;
            int n = 0; while (*pp && *pp != '"' && n < 64) d.hash[n++] = *pp++;
            d.hash[n] = 0; }
        }

        pp = strstr(json, "\"genome\"");
        if (pp) {
            pp = strchr(pp + 8, '"'); if (pp) { pp++;
            int n = 0; while (*pp && *pp != '"' && n < 64) d.genome[n++] = *pp++;
            d.genome[n] = 0; }
        }

        bwave_lcd_set_declared(&d);

        uint8_t hash_bytes[32] = {0};
        for (int hi = 0; hi < 32 && d.hash[hi*2] && d.hash[hi*2+1]; hi++) {
            char hb[3] = { d.hash[hi*2], d.hash[hi*2+1], 0 };
            hash_bytes[hi] = (uint8_t)strtol(hb, NULL, 16);
        }
        bwave_identity_save_to_sd(d.position, d.element,
                                  d.inverse, d.ray,
                                  hash_bytes, d.genome);

        snprintf(buf, buflen,
            "{\"ok\":true,\"cmd\":\"declare\","
            "\"name\":\"%s\",\"pos\":%u,\"ray\":%u,"
            "\"inv\":%u,\"layers\":%u}\n",
            d.name, (unsigned)d.position, (unsigned)d.ray,
            (unsigned)d.inverse, (unsigned)d.layers_measured);
        ESP_LOGI(TAG, "DECLARED: %s pos=%u ray=%u", d.name, d.position, d.ray);

    } else if (strcmp(cmd, "mirror") == 0) {
        bwave_cascade_mirror_handle(json, buf, buflen);

    } else if (strcmp(cmd, "store_fold") == 0) {
        bwave_cascade_store_fold_handle(json, buf, buflen);

    } else if (strcmp(cmd, "mirror_broadcast") == 0) {
        bwave_cascade_broadcast_sd(buf, buflen);

    } else if (strcmp(cmd, "field_analysis") == 0) {
        bwave_cascade_field_analysis(buf, buflen);

    } else if (strcmp(cmd, "flip_status") == 0) {
        const bwave_field_state_t *fs = bwave_cascade_get_field();
        int top_pos[8] = {0};
        float top_e[8] = {0};
        for (int pp = 1; pp < GF_P; pp++) {
            if (fs->energy[pp] > top_e[7]) {
                top_e[7] = fs->energy[pp];
                top_pos[7] = pp;
                for (int k = 6; k >= 0; k--) {
                    if (top_e[k+1] > top_e[k]) {
                        float te = top_e[k]; top_e[k] = top_e[k+1]; top_e[k+1] = te;
                        int tp = top_pos[k]; top_pos[k] = top_pos[k+1]; top_pos[k+1] = tp;
                    }
                }
            }
        }
        int o = snprintf(buf, buflen,
            "{\"ok\":true,\"cmd\":\"flip_status\","
            "\"xray\":%.3f,\"flip_state\":%u,"
            "\"flips\":{\"F\":%lu,\"T\":%lu,\"PM\":%lu},"
            "\"nano2\":{\"active\":%s",
            fs->xray, fs->flip_state,
            (unsigned long)fs->flip_count[FLIP_F],
            (unsigned long)fs->flip_count[FLIP_T],
            (unsigned long)fs->flip_count[FLIP_PM],
            fs->nano2_active ? "true" : "false");
        if (fs->nano2_active) {
            o += snprintf(buf + o, buflen - o,
                ",\"pub\":%u,\"inv\":%u,\"fold\":%u,\"peer\":%u,"
                "\"name\":\"%s\"",
                fs->nano2_pub, fs->nano2_inv,
                fs->nano2_fold, fs->nano2_peer,
                fs->nano2_name);
        }
        o += snprintf(buf + o, buflen - o, "},\"top\":[");
        for (int k = 0; k < 8 && top_pos[k] > 0; k++) {
            if (k) buf[o++] = ',';
            uint8_t pi = bwave_cascade_tl_op(top_pos[k]);
            o += snprintf(buf + o, buflen - o,
                "{\"pos\":%d,\"e\":%.2f,\"dlog\":%u,\"ray\":%u,"
                "\"qr\":%s,\"op\":\"%s\",\"tl\":\"%s\"}",
                top_pos[k], top_e[k],
                bwave_cascade_dlog(top_pos[k]),
                bwave_cascade_ray(top_pos[k]),
                bwave_cascade_is_qr(top_pos[k]) ? "true" : "false",
                pi < 10 ? (const char *[]){"NOP","IDENT","EXIST","SEED",
                    "SHIFT","STORE","FOLD","JUMP","SYNC","HALT"}[pi] : "?",
                bwave_cascade_tl_name(pi));
        }
        o += snprintf(buf + o, buflen - o, "]}\n");

    } else if (strcmp(cmd, "slip_status") == 0 || strcmp(cmd, "serial_status") == 0) {
        snprintf(buf, buflen,
            "{\"ok\":true,\"cmd\":\"serial_status\","
            "\"serial_up\":%s}\n",
            bwave_slip_is_up() ? "true" : "false");

    } else if (strcmp(cmd, "doppler") == 0) {
        float vel[DSP_TOP_K];
        int count = 0;
        bwave_dsp_get_doppler(vel, &count);
        int o = snprintf(buf, buflen, "{\"ok\":true,\"cmd\":\"doppler\",\"count\":%d,\"velocity\":[", count);
        for (int vi = 0; vi < count; vi++) {
            if (vi) buf[o++] = ',';
            o += snprintf(buf + o, buflen - o, "%.4f", vel[vi]);
        }
        o += snprintf(buf + o, buflen - o, "]}\n");

    } else if (strcmp(cmd, "persons") == 0) {
        bwave_person_pkt_t pkt;
        int np = bwave_dsp_get_persons(&pkt);
        int o = snprintf(buf, buflen,
            "{\"ok\":true,\"cmd\":\"persons\",\"n_persons\":%d,\"persons\":[", np);
        for (int pi = 0; pi < BWAVE_MAX_PERSONS; pi++) {
            if (pi) buf[o++] = ',';
            o += snprintf(buf + o, buflen - o,
                "{\"active\":%s,\"br\":%.2f,\"hr\":%.2f,\"sc\":%u}",
                pkt.persons[pi].active ? "true" : "false",
                pkt.persons[pi].breathing_rate / 100.0f,
                pkt.persons[pi].heartrate / 100.0f,
                (unsigned)pkt.persons[pi].subcarrier_idx);
        }
        o += snprintf(buf + o, buflen - o, "]}\n");

    } else if (strcmp(cmd, "features") == 0) {
        float feat[8];
        int fc = 0;
        bwave_dsp_get_features(feat, &fc);
        int o = snprintf(buf, buflen,
            "{\"ok\":true,\"cmd\":\"features\",\"labels\":"
            "[\"presence\",\"motion\",\"br_norm\",\"hr_norm\","
            "\"sc_var\",\"n_persons\",\"fall\",\"rssi_norm\"],"
            "\"values\":[");
        for (int fi = 0; fi < fc; fi++) {
            if (fi) buf[o++] = ',';
            o += snprintf(buf + o, buflen - o, "%.4f", feat[fi]);
        }
        o += snprintf(buf + o, buflen - o, "]}\n");

    } else if (strcmp(cmd, "resonance") == 0) {
        bwave_dsp_resonance_t r;
        bwave_dsp_filter_t f;
        bwave_dsp_get_filter(&f);
        if (!bwave_dsp_get_resonance(&r)) {
            int o = snprintf(buf, buflen,
                "{\"ok\":true,\"cmd\":\"resonance\",\"ready\":false,");
            o += filter_json(buf + o, buflen - o, &f);
            snprintf(buf + o, buflen - o, "}\n");
        } else {
            /* eq. 24: how far each filter passes the other band's rhythm */
            float hr_at_br = r.br.valid
                ? 20.0f * log10f(bwave_res_selectivity(r.br.f0_hz, f.hr_f0, f.hr_q, f.hr_stages) + 1e-9f) : 0.0f;
            float br_at_hr = r.hr.valid
                ? 20.0f * log10f(bwave_res_selectivity(r.hr.f0_hz, f.br_f0, f.br_q, f.br_stages) + 1e-9f) : 0.0f;
            int o = snprintf(buf, buflen,
                "{\"ok\":true,\"cmd\":\"resonance\",\"ready\":true,"
                "\"n\":%u,\"age_ms\":%lu,",
                (unsigned)r.n_samples,
                (unsigned long)((uint32_t)(esp_timer_get_time() / 1000) - r.updated_ms));
            o += res_peak_json(buf + o, buflen - o, "br", &r.br);
            buf[o++] = ',';
            o += res_peak_json(buf + o, buflen - o, "hr", &r.hr);
            buf[o++] = ',';
            o += res_peak_json(buf + o, buflen - o, "am_br", &r.am_br);
            o += snprintf(buf + o, buflen - o,
                ",\"br_decrement\":%.4f,\"br_q_decrement\":%.2f,\"br_cycles\":%u,"
                "\"hr_decrement\":%.4f,\"hr_q_decrement\":%.2f,\"hr_cycles\":%u,"
                "\"am_depth\":%.4f,\"pm_index\":%.4f,\"am_pm_agree\":%s,"
                "\"hr_filter_at_br_db\":%.1f,\"br_filter_at_hr_db\":%.1f,",
                r.br_decrement, bwave_res_q_from_decrement(r.br_decrement), r.br_cycles,
                r.hr_decrement, bwave_res_q_from_decrement(r.hr_decrement), r.hr_cycles,
                r.am_depth, r.pm_index, r.am_pm_agree ? "true" : "false",
                hr_at_br, br_at_hr);
            o += filter_json(buf + o, buflen - o, &f);
            snprintf(buf + o, buflen - o, "}\n");
        }

    } else if (strcmp(cmd, "filter") == 0) {
        /* {"cmd":"filter","br_q":2.0,"hr_q":0,"br_stages":2}
           omitted = keep, q 0 = band default; not persisted (use NVS) */
        float br_q = -1.0f, hr_q = -1.0f, bs = 0.0f, hs = 0.0f;
        json_num(json, "br_q", &br_q);
        json_num(json, "hr_q", &hr_q);
        json_num(json, "br_stages", &bs);
        json_num(json, "hr_stages", &hs);
        if ((br_q > 0.0f && br_q < 0.05f) || br_q > 50.0f ||
            (hr_q > 0.0f && hr_q < 0.05f) || hr_q > 50.0f ||
            bs < 0.0f || bs > BWAVE_RES_MAX_STAGES ||
            hs < 0.0f || hs > BWAVE_RES_MAX_STAGES) {
            snprintf(buf, buflen,
                "{\"ok\":false,\"cmd\":\"filter\","
                "\"err\":\"q must be 0 or 0.05-50, stages 1-%d\"}\n",
                BWAVE_RES_MAX_STAGES);
        } else {
            bwave_dsp_set_filter(br_q, hr_q, (uint8_t)bs, (uint8_t)hs);
            bwave_dsp_filter_t f;
            bwave_dsp_get_filter(&f);
            int o = snprintf(buf, buflen, "{\"ok\":true,\"cmd\":\"filter\",");
            o += filter_json(buf + o, buflen - o, &f);
            snprintf(buf + o, buflen - o, "}\n");
        }

    } else if (strcmp(cmd, "csi_macs") == 0) {
        uint8_t macs[32][6];
        int8_t  rssi[32];
        uint16_t counts[32];
        int nm = bwave_csi_get_mac_table(macs, rssi, counts, 32);
        int o = snprintf(buf, buflen,
            "{\"ok\":true,\"cmd\":\"csi_macs\",\"count\":%d,\"macs\":[", nm);
        for (int mi = 0; mi < nm; mi++) {
            if (mi) buf[o++] = ',';
            o += snprintf(buf + o, buflen - o,
                "{\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
                "\"rssi\":%d,\"frames\":%u}",
                macs[mi][0], macs[mi][1], macs[mi][2],
                macs[mi][3], macs[mi][4], macs[mi][5],
                (int)rssi[mi], (unsigned)counts[mi]);
        }
        o += snprintf(buf + o, buflen - o, "]}\n");

    } else if (strcmp(cmd, "status") == 0) {
        bwave_csi_snapshot_t snap;
        bwave_csi_get_snapshot(&snap);
        const bwave_field_state_t *fs = bwave_cascade_get_field();
        bwave_vitals_pkt_t vpkt;
        bool has_v = bwave_dsp_get_vitals(&vpkt);
        float delta, theta, alpha;
        bwave_dsp_get_brainwave(&delta, &theta, &alpha);
        int o = snprintf(buf, buflen,
            "{\"ok\":true,\"cmd\":\"status\","
            "\"node\":%u,\"rssi\":%d,\"ch\":%d,"
            "\"cb\":%lu,\"iq_len\":%d,"
            "\"sd\":%s,\"slip\":%s,"
            "\"flip\":%u,\"xray\":%.3f,"
            "\"nano2\":%s,"
            "\"fall\":%s,"
            "\"presence\":%.4f,"
            "\"motion\":%.4f",
            bwave_csi_get_node_id(),
            snap.rssi, snap.channel,
            (unsigned long)snap.cb_count, snap.iq_len,
            bwave_sd_is_mounted() ? "true" : "false",
            bwave_slip_is_up() ? "true" : "false",
            fs->flip_state, fs->xray,
            fs->nano2_active ? "true" : "false",
            bwave_dsp_get_fall() ? "true" : "false",
            bwave_dsp_get_presence(),
            bwave_dsp_get_motion());
        if (has_v) {
            o += snprintf(buf + o, buflen - o,
                ",\"br\":%.2f,\"hr\":%.2f,"
                "\"n_persons\":%u,\"temp\":%d",
                vpkt.breathing_rate / 100.0f,
                vpkt.heartrate / 10000.0f,
                (unsigned)vpkt.n_persons,
                vpkt.die_temp_c);
        }
        o += snprintf(buf + o, buflen - o,
            ",\"delta\":%.6f,\"theta\":%.6f,\"alpha\":%.6f",
            delta, theta, alpha);
        o += snprintf(buf + o, buflen - o, "}\n");

    } else if (strcmp(cmd, "ble_field") == 0) {
        const bwave_ble_device_t *devs = bwave_ble_get_devices();
        int ndev = bwave_ble_get_device_count();
        const bwave_field_state_t *fs = bwave_cascade_get_field();
        uint16_t prods[16];
        int actual = 0;
        int o = snprintf(buf, buflen,
            "{\"ok\":true,\"cmd\":\"ble_field\",\"count\":%d,\"devices\":[", ndev);
        for (int bi = 0; bi < ndev && bi < 16 && o < (int)buflen - 400; bi++) {
            if (bi) buf[o++] = ',';
            uint8_t *m = (uint8_t *)devs[bi].mac;
            uint16_t mac_pos[6];
            uint32_t prod = 1;
            for (int mb = 0; mb < 6; mb++) {
                mac_pos[mb] = (m[mb] + 1) % GF_P;
                if (mac_pos[mb] == 0) mac_pos[mb] = 1;
                prod = (prod * mac_pos[mb]) % GF_P;
            }
            prods[bi] = (uint16_t)prod;
            actual = bi + 1;
            uint16_t fp = (uint16_t)(GF_P - prod);
            uint16_t inv = bwave_cascade_inv((uint16_t)prod);
            uint16_t imp = (uint16_t)((prod * 120UL) % GF_P);
            uint16_t dl = bwave_cascade_dlog((uint16_t)prod);
            o += snprintf(buf + o, buflen - o,
                "{\"name\":\"%.12s\",\"rssi\":%d,\"pos\":%u,"
                "\"prod\":%lu,\"dlog\":%u,\"ray\":%u,"
                "\"qr\":%s,\"fold\":%u,\"inv\":%u,"
                "\"imp\":%u,\"mirrored\":%u,\"energy\":%.2f}",
                devs[bi].name, (int)devs[bi].rssi,
                (unsigned)devs[bi].position,
                (unsigned long)prod, dl, dl % 12,
                (dl % 2 == 0) ? "true" : "false",
                fp, inv, imp,
                (unsigned)devs[bi].mirrored,
                fs->energy[prod < GF_P ? prod : 1]);
        }
        o += snprintf(buf + o, buflen - o, "],\"fold_pairs\":[");
        int fc = 0;
        for (int i = 0; i < actual && o < (int)buflen - 200; i++) {
            for (int j = i + 1; j < actual; j++) {
                if (prods[i] + prods[j] == GF_P) {
                    if (fc) buf[o++] = ',';
                    uint16_t ia = bwave_cascade_impedance(prods[i]);
                    uint16_t ib = bwave_cascade_impedance(prods[j]);
                    o += snprintf(buf + o, buflen - o,
                        "{\"a\":%d,\"b\":%d,\"pa\":%u,\"pb\":%u,"
                        "\"imp_a\":%u,\"imp_b\":%u,\"imp_fold\":%s}",
                        i, j, prods[i], prods[j],
                        ia, ib,
                        (ia + ib == GF_P) ? "true" : "false");
                    fc++;
                    uint16_t fp8[8] = {
                        prods[i], prods[j], ia, ib,
                        bwave_cascade_inv(prods[i]),
                        bwave_cascade_inv(prods[j]),
                        bwave_cascade_reflection(prods[i]),
                        bwave_cascade_reflection(prods[j])
                    };
                    char lbl[29];
                    snprintf(lbl, sizeof(lbl), "BF:%u+%u",
                        (unsigned)prods[i], (unsigned)prods[j]);
                    bwave_cascade_store_device(lbl, fp8, 8);
                }
            }
        }
        o += snprintf(buf + o, buflen - o, "],\"inv_pairs\":[");
        int ic = 0;
        for (int i = 0; i < actual && o < (int)buflen - 200; i++) {
            for (int j = i + 1; j < actual; j++) {
                if (((uint32_t)prods[i] * prods[j]) % GF_P == 1) {
                    if (ic) buf[o++] = ',';
                    o += snprintf(buf + o, buflen - o,
                        "{\"a\":%d,\"b\":%d,\"pa\":%u,\"pb\":%u}",
                        i, j, prods[i], prods[j]);
                    ic++;
                    uint16_t ip2[2] = { prods[i], prods[j] };
                    char lbl[29];
                    snprintf(lbl, sizeof(lbl), "BI:%u*%u",
                        (unsigned)prods[i], (unsigned)prods[j]);
                    bwave_cascade_store_device(lbl, ip2, 2);
                }
            }
        }
        o += snprintf(buf + o, buflen - o, "]}\n");

    } else if (strcmp(cmd, "config") == 0) {
        extern bwave_config_t g_bwave_config;
        snprintf(buf, buflen,
            "{\"ok\":true,\"cmd\":\"config\","
            "\"ssid\":\"%s\",\"target\":\"%s:%u\","
            "\"node\":%u,\"ch\":%u,\"tier\":%u,"
            "\"vital_ms\":%u,\"presence_thresh\":%.2f,"
            "\"top_k\":%u,\"sd\":%u,\"lcd\":%u,"
            "\"br_q\":%.2f,\"hr_q\":%.2f,\"br_stages\":%u,\"hr_stages\":%u}\n",
            g_bwave_config.wifi_ssid,
            g_bwave_config.target_ip, g_bwave_config.target_port,
            g_bwave_config.node_id, g_bwave_config.wifi_channel,
            g_bwave_config.edge_tier,
            g_bwave_config.vital_interval_ms,
            g_bwave_config.presence_thresh,
            g_bwave_config.top_k_count,
            g_bwave_config.sd_logging,
            g_bwave_config.lcd_enabled,
            g_bwave_config.br_q, g_bwave_config.hr_q,
            g_bwave_config.br_stages, g_bwave_config.hr_stages);

    } else if (strcmp(cmd, "lcd") == 0) {
        const bwave_declared_t *d = bwave_lcd_get_declared();
        if (d && d->active) {
            snprintf(buf, buflen,
                "{\"ok\":true,\"cmd\":\"lcd\",\"active\":true,"
                "\"name\":\"%s\",\"pos\":%u,\"element\":%u,"
                "\"ray\":%u,\"inv\":%u,\"dlog\":%u,"
                "\"layers\":%u,\"genome\":\"%s\"}\n",
                d->name, (unsigned)d->position,
                (unsigned)d->element, (unsigned)d->ray,
                (unsigned)d->inverse, (unsigned)d->dlog,
                (unsigned)d->layers_measured, d->genome);
        } else {
            snprintf(buf, buflen,
                "{\"ok\":true,\"cmd\":\"lcd\",\"active\":false}\n");
        }

    } else {
        snprintf(buf, buflen, "{\"ok\":false,\"err\":\"unknown: %s\"}\n", cmd);
    }
}

/* ── public entry point for UDP cmd server and any caller ── */

void bwave_cmd_handle_line(const char *line, char *resp, size_t resp_len)
{
    resp[0] = '\0';

    if (line[0] == '$' && strlen(line) >= 3) {
        switch (line[1]) {
        case 'X': bwave_cascade_handle_xray(line); break;
        case 'I': bwave_cascade_handle_identity_line(line); break;
        case 'U': bwave_cascade_handle_upload_line(line); break;
        case 'F': bwave_cascade_handle_field_line(line); break;
        case 'G': bwave_cascade_handle_genome_line(line); break;
        case 'D': bwave_cascade_handle_doubling_line(line); break;
        case 'Q': handle_tunnel_line(line); return;
        default: break;
        }
        snprintf(resp, resp_len, "{\"ok\":true}\n");
        return;
    }

    if (strstr(line, "\"cmd\"")) {
        handle_cmd(line, resp, resp_len);
    }
}

void bwave_cmd_init(void)
{
    ESP_LOGI(TAG, "Command handler ready (UDP via SLIP)");
}
