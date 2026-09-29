#include "bwave_dsp.h"
#include "bwave_csi.h"
#include "bwave_stream.h"

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#ifdef CONFIG_SOC_TEMP_SENSOR_SUPPORTED
#include "driver/temperature_sensor.h"
static temperature_sensor_handle_t s_temp_handle = NULL;
#endif
static float s_die_temp_c = 25.0f;

static const char *TAG = "bwave_dsp";

/* SPSC ring buffer */
typedef struct {
    uint8_t  iq_data[DSP_MAX_IQ_BYTES];
    uint16_t iq_len;
    int8_t   rssi;
    uint8_t  channel;
    uint32_t timestamp_us;
} dsp_ring_slot_t;

static struct {
    dsp_ring_slot_t slots[DSP_RING_SLOTS];
    volatile uint32_t head;
    volatile uint32_t tail;
} s_ring;

static uint32_t s_ring_drops;

static inline bool ring_push(const uint8_t *iq, uint16_t len,
                             int8_t rssi, uint8_t channel)
{
    uint32_t next = (s_ring.head + 1) % DSP_RING_SLOTS;
    if (next == s_ring.tail) { s_ring_drops++; return false; }

    dsp_ring_slot_t *slot = &s_ring.slots[s_ring.head];
    uint16_t copy = (len > DSP_MAX_IQ_BYTES) ? DSP_MAX_IQ_BYTES : len;
    memcpy(slot->iq_data, iq, copy);
    slot->iq_len = copy;
    slot->rssi = rssi;
    slot->channel = channel;
    slot->timestamp_us = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);

    __sync_synchronize();
    s_ring.head = next;
    return true;
}

static inline bool ring_pop(dsp_ring_slot_t *out)
{
    if (s_ring.tail == s_ring.head) return false;
    memcpy(out, &s_ring.slots[s_ring.tail], sizeof(dsp_ring_slot_t));
    __sync_synchronize();
    s_ring.tail = (s_ring.tail + 1) % DSP_RING_SLOTS;
    return true;
}

/* Biquad IIR */
static void biquad_design(bwave_biquad_t *bq, float fs, float f_lo, float f_hi)
{
    float w0 = 2.0f * M_PI * (f_lo + f_hi) / 2.0f / fs;
    float bw = 2.0f * M_PI * (f_hi - f_lo) / fs;
    float alpha = sinf(w0) * sinhf(logf(2.0f) / 2.0f * bw / sinf(w0));
    float a0_inv = 1.0f / (1.0f + alpha);
    bq->b0 =  alpha * a0_inv;
    bq->b1 =  0.0f;
    bq->b2 = -alpha * a0_inv;
    bq->a1 = -2.0f * cosf(w0) * a0_inv;
    bq->a2 =  (1.0f - alpha) * a0_inv;
    bq->x1 = bq->x2 = bq->y1 = bq->y2 = 0.0f;
}

static inline float biquad_process(bwave_biquad_t *bq, float x)
{
    float y = bq->b0 * x + bq->b1 * bq->x1 + bq->b2 * bq->x2
            - bq->a1 * bq->y1 - bq->a2 * bq->y2;
    bq->x2 = bq->x1; bq->x1 = x;
    bq->y2 = bq->y1; bq->y1 = y;
    return y;
}

/* Phase extraction */
static inline float extract_phase(const uint8_t *iq, uint16_t idx)
{
    int8_t i_val = (int8_t)iq[idx * 2];
    int8_t q_val = (int8_t)iq[idx * 2 + 1];
    return atan2f((float)q_val, (float)i_val);
}

static inline float unwrap_phase(float prev, float curr)
{
    float diff = curr - prev;
    if (diff > M_PI)       diff -= 2.0f * M_PI;
    else if (diff < -M_PI) diff += 2.0f * M_PI;
    return prev + diff;
}

/* Welford running statistics */
typedef struct { double mean, m2; uint32_t count; } welford_t;

static inline void welford_update(welford_t *w, double x)
{
    w->count++;
    double d1 = x - w->mean;
    w->mean += d1 / (double)w->count;
    double d2 = x - w->mean;
    w->m2 += d1 * d2;
}

static inline double welford_var(const welford_t *w)
{
    return (w->count > 1) ? (w->m2 / (double)(w->count - 1)) : 0.0;
}

/* Zero-crossing BPM */
static float estimate_bpm(const float *history, uint16_t len, float sample_rate)
{
    if (len < 4) return 0.0f;

    uint16_t crossings[128];
    uint16_t n_cross = 0;

    for (uint16_t i = 1; i < len && n_cross < 128; i++) {
        if (history[i - 1] <= 0.0f && history[i] > 0.0f)
            crossings[n_cross++] = i;
    }

    if (n_cross < 2) return 0.0f;

    float total = 0.0f;
    for (uint16_t i = 1; i < n_cross; i++)
        total += (float)(crossings[i] - crossings[i - 1]);
    float avg = total / (float)(n_cross - 1);

    if (avg < 1.0f) return 0.0f;
    return (sample_rate / avg) * 60.0f;
}

static int hr_is_breath_harmonic(float hr, float br)
{
    if (br <= 0.0f || hr <= 0.0f) return 0;
    for (int k = 2; k <= 3; k++)
        if (fabsf(hr - (float)k * br) < 0.12f * hr) return 1;
    return 0;
}

/* DSP state */
static bwave_dsp_config_t s_cfg;
static welford_t s_sc_var[DSP_MAX_SUBCARRIERS];
static float s_prev_phase[DSP_MAX_SUBCARRIERS];
static bool  s_phase_init;
static uint8_t s_top_k[DSP_TOP_K];
static uint8_t s_top_k_count;

static float s_phase_history[DSP_PHASE_HISTORY];
static uint16_t s_hist_len, s_hist_idx;

static bwave_biquad_t s_bq_br, s_bq_hr;
static bwave_biquad_t s_bq_delta, s_bq_theta, s_bq_alpha;

static float s_delta_energy, s_theta_energy, s_alpha_energy;
#define SMOOTH 0.95f

static float s_br_filt[DSP_PHASE_HISTORY];
static float s_hr_filt[DSP_PHASE_HISTORY];
static float s_scratch_br[DSP_PHASE_HISTORY];
static float s_scratch_hr[DSP_PHASE_HISTORY];

static float s_breathing_bpm, s_heartrate_bpm;
static float s_motion_energy, s_presence_score;
static bool  s_presence_detected;
static int8_t s_latest_rssi;
static uint32_t s_frame_count;
static float s_breath_ref;

static int64_t s_last_vitals_send_us;
static volatile bwave_vitals_pkt_t s_latest_pkt;
static volatile bool s_pkt_valid;

/* Doppler velocity */
static float s_topk_velocity[DSP_TOP_K];
static float s_prev_topk_phase[DSP_TOP_K];
static bool  s_topk_phase_init;

/* Multi-person */
typedef struct {
    float    phase_history[DSP_PHASE_HISTORY];
    uint16_t history_len;
    uint16_t history_idx;
    float    breathing_bpm;
    float    heartrate_bpm;
    uint8_t  subcarrier_idx;
    bool     active;
} bwave_person_vitals_t;

static bwave_person_vitals_t s_persons[BWAVE_MAX_PERSONS];
static bwave_biquad_t s_person_bq_br[BWAVE_MAX_PERSONS];
static bwave_biquad_t s_person_bq_hr[BWAVE_MAX_PERSONS];
static float s_person_br_filt[BWAVE_MAX_PERSONS][DSP_PHASE_HISTORY];
static float s_person_hr_filt[BWAVE_MAX_PERSONS][DSP_PHASE_HISTORY];
static float s_person_breath_ref[BWAVE_MAX_PERSONS];

/* Feature vector */
static uint16_t s_feature_seq;

/* Fall detection */
static bool  s_fall_detected;
static float s_prev_phase_velocity;
static uint8_t s_fall_consec_count;
static int64_t s_fall_last_alert_us;
#define FALL_COOLDOWN_MS 5000
#define FALL_CONSEC_MIN  3

/* Amplitude baseline */
#define AMP_BASELINE_ALPHA  0.02f
#define AMP_BASELINE_INTERVAL_US (30 * 1000000LL)
static float s_amp_baseline[BWAVE_BASELINE_MAX_SC];
static bool  s_amp_baseline_valid;
static int64_t s_last_baseline_send_us;

/* Top-K selection */
static void update_top_k(uint16_t n_sub)
{
    uint8_t k = s_cfg.top_k_count;
    if (k > DSP_TOP_K) k = DSP_TOP_K;
    if (k > n_sub) k = (uint8_t)n_sub;

    bool used[DSP_MAX_SUBCARRIERS];
    memset(used, 0, sizeof(used));

    for (uint8_t ki = 0; ki < k; ki++) {
        double best = -1.0;
        uint8_t best_idx = 0;
        for (uint16_t sc = 0; sc < n_sub; sc++) {
            if (used[sc]) continue;
            double v = welford_var(&s_sc_var[sc]);
            if (v > best) { best = v; best_idx = (uint8_t)sc; }
        }
        s_top_k[ki] = best_idx;
        used[best_idx] = true;
    }
    s_top_k_count = k;
}

/* Send packets */
static void send_vitals(void)
{
    bwave_vitals_pkt_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = BWAVE_VITALS_MAGIC;
    pkt.node_id = bwave_csi_get_node_id();
    pkt.flags = (s_presence_detected ? 1 : 0);
    pkt.breathing_rate = (uint16_t)(s_breathing_bpm * 100.0f);
    pkt.heartrate = (uint32_t)(s_heartrate_bpm * 10000.0f);
    pkt.rssi = s_latest_rssi;
    pkt.die_temp_c = (int8_t)s_die_temp_c;
    pkt.motion_energy = s_motion_energy;
    pkt.presence_score = s_presence_score;
    pkt.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);

    bwave_stream_send((const uint8_t *)&pkt, sizeof(pkt));

    s_latest_pkt = pkt;
    s_pkt_valid = true;
}

static void send_brainwave(void)
{
    bwave_band_pkt_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = BWAVE_BWAVE_MAGIC;
    pkt.node_id = bwave_csi_get_node_id();
    pkt.timestamp_ms = (uint16_t)((esp_timer_get_time() / 1000) & 0xFFFF);
    pkt.delta = s_delta_energy;
    pkt.theta = s_theta_energy;
    pkt.alpha = s_alpha_energy;

    bwave_stream_send((const uint8_t *)&pkt, sizeof(pkt));
}

static void send_feature_vector(void)
{
    bwave_feature_pkt_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = BWAVE_FEATURE_MAGIC;
    pkt.node_id = bwave_csi_get_node_id();
    pkt.seq = s_feature_seq++;
    pkt.timestamp_us = esp_timer_get_time();

    float p = s_presence_score;
    pkt.features[0] = p > 10.0f ? 1.0f : (p < 0.0f ? 0.0f : p / 10.0f);
    float m = s_motion_energy;
    pkt.features[1] = m > 10.0f ? 1.0f : (m < 0.0f ? 0.0f : m / 10.0f);
    pkt.features[2] = s_breathing_bpm > 0.0f
        ? (s_breathing_bpm / 30.0f > 1.0f ? 1.0f : s_breathing_bpm / 30.0f) : 0.0f;
    pkt.features[3] = s_heartrate_bpm > 0.0f
        ? (s_heartrate_bpm / 120.0f > 1.0f ? 1.0f : s_heartrate_bpm / 120.0f) : 0.0f;

    float var_mean = 0.0f;
    if (s_top_k_count > 0) {
        float var_sum = 0.0f;
        uint8_t k = s_top_k_count < DSP_TOP_K ? s_top_k_count : DSP_TOP_K;
        for (uint8_t i = 0; i < k; i++)
            var_sum += (float)welford_var(&s_sc_var[s_top_k[i]]);
        var_mean = var_sum / (float)k;
    }
    pkt.features[4] = var_mean > 1.0f ? 1.0f : (var_mean < 0.0f ? 0.0f : var_mean);

    uint8_t n_active = 0;
    for (uint8_t i = 0; i < BWAVE_MAX_PERSONS; i++)
        if (s_persons[i].active) n_active++;
    pkt.features[5] = (float)n_active / 4.0f;
    if (pkt.features[5] > 1.0f) pkt.features[5] = 1.0f;

    pkt.features[6] = s_fall_detected ? 1.0f : 0.0f;

    pkt.features[7] = ((float)s_latest_rssi + 100.0f) / 100.0f;
    if (pkt.features[7] > 1.0f) pkt.features[7] = 1.0f;
    if (pkt.features[7] < 0.0f) pkt.features[7] = 0.0f;

    bwave_stream_send((const uint8_t *)&pkt, sizeof(pkt));
}

static void send_person_vitals(void)
{
    bwave_person_pkt_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = BWAVE_PERSON_MAGIC;
    pkt.node_id = bwave_csi_get_node_id();
    pkt.timestamp_ms = (uint16_t)((esp_timer_get_time() / 1000) & 0xFFFF);
    uint8_t n = 0;
    for (uint8_t p = 0; p < BWAVE_MAX_PERSONS; p++) {
        pkt.persons[p].active = s_persons[p].active ? 1 : 0;
        if (s_persons[p].active) {
            pkt.persons[p].breathing_rate = (uint16_t)(s_persons[p].breathing_bpm * 100.0f);
            pkt.persons[p].heartrate = (uint16_t)(s_persons[p].heartrate_bpm * 100.0f);
            pkt.persons[p].subcarrier_idx = s_persons[p].subcarrier_idx;
            n++;
        }
    }
    pkt.n_persons = n;
    bwave_stream_send((const uint8_t *)&pkt, sizeof(pkt));
}

static void send_doppler(void)
{
    bwave_doppler_pkt_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = BWAVE_DOPPLER_MAGIC;
    pkt.node_id = bwave_csi_get_node_id();
    pkt.n_carriers = s_top_k_count < DSP_TOP_K ? s_top_k_count : DSP_TOP_K;
    pkt.timestamp_ms = (uint16_t)((esp_timer_get_time() / 1000) & 0xFFFF);
    for (uint8_t i = 0; i < pkt.n_carriers; i++)
        pkt.velocity[i] = s_topk_velocity[i];
    bwave_stream_send((const uint8_t *)&pkt, sizeof(pkt));
}

static void update_amp_baseline(const uint8_t *iq_data, uint16_t n_sc)
{
    if (n_sc == 0 || n_sc > BWAVE_BASELINE_MAX_SC) return;
    for (uint16_t k = 0; k < n_sc; k++) {
        int8_t i_val = (int8_t)iq_data[2 * k];
        int8_t q_val = (int8_t)iq_data[2 * k + 1];
        float a = sqrtf((float)(i_val * i_val + q_val * q_val));
        if (!s_amp_baseline_valid)
            s_amp_baseline[k] = a;
        else
            s_amp_baseline[k] = s_amp_baseline[k] * (1.0f - AMP_BASELINE_ALPHA)
                               + a * AMP_BASELINE_ALPHA;
    }
    s_amp_baseline_valid = true;
}

static void send_amp_baseline(uint16_t n_sc)
{
    if (!s_amp_baseline_valid || n_sc == 0) return;
    int64_t now = esp_timer_get_time();
    if ((now - s_last_baseline_send_us) < AMP_BASELINE_INTERVAL_US) return;
    s_last_baseline_send_us = now;

    uint16_t sc = (n_sc > BWAVE_BASELINE_MAX_SC) ? BWAVE_BASELINE_MAX_SC : n_sc;
    size_t pkt_len = 8 + sc * 2;
    static uint8_t pkt[8 + BWAVE_BASELINE_MAX_SC * 2];
    uint32_t magic = BWAVE_BASELINE_MAGIC;
    memcpy(&pkt[0], &magic, 4);
    pkt[4] = bwave_csi_get_node_id();
    pkt[5] = (uint8_t)(int8_t)s_die_temp_c;
    memcpy(&pkt[6], &sc, 2);
    for (uint16_t k = 0; k < sc; k++) {
        uint16_t val = (uint16_t)(s_amp_baseline[k] * 100.0f);
        memcpy(&pkt[8 + k * 2], &val, 2);
    }
    bwave_stream_send(pkt, pkt_len);
}

static void update_multi_person(const uint8_t *iq_data, uint16_t n_sc, float sample_rate)
{
    if (s_top_k_count < 2) return;

    uint8_t n_persons = s_top_k_count / 2;
    if (n_persons > BWAVE_MAX_PERSONS) n_persons = BWAVE_MAX_PERSONS;
    if (n_persons < 1) n_persons = 1;

    uint8_t subs_per = s_top_k_count / n_persons;

    for (uint8_t p = 0; p < n_persons; p++) {
        bwave_person_vitals_t *pv = &s_persons[p];
        pv->active = true;
        pv->subcarrier_idx = s_top_k[p * subs_per];

        float avg_phase = 0.0f;
        uint8_t count = 0;
        for (uint8_t s = 0; s < subs_per; s++) {
            uint8_t sc_idx = s_top_k[p * subs_per + s];
            if (sc_idx < n_sc) {
                avg_phase += extract_phase(iq_data, sc_idx);
                count++;
            }
        }
        if (count > 0) avg_phase /= (float)count;

        if (pv->history_len > 0) {
            uint16_t prev_idx = (pv->history_idx + DSP_PHASE_HISTORY - 1) % DSP_PHASE_HISTORY;
            avg_phase = unwrap_phase(pv->phase_history[prev_idx], avg_phase);
        }

        pv->phase_history[pv->history_idx] = avg_phase;
        pv->history_idx = (pv->history_idx + 1) % DSP_PHASE_HISTORY;
        if (pv->history_len < DSP_PHASE_HISTORY) pv->history_len++;

        float br_val = biquad_process(&s_person_bq_br[p], avg_phase);
        float hr_val = biquad_process(&s_person_bq_hr[p], avg_phase);

        uint16_t idx = (pv->history_idx + DSP_PHASE_HISTORY - 1) % DSP_PHASE_HISTORY;
        s_person_br_filt[p][idx] = br_val;
        s_person_hr_filt[p][idx] = hr_val;

        if (pv->history_len >= 64) {
            uint16_t buf_len = pv->history_len;
            for (uint16_t i = 0; i < buf_len; i++) {
                uint16_t ri = (pv->history_idx + DSP_PHASE_HISTORY - buf_len + i) % DSP_PHASE_HISTORY;
                s_scratch_br[i] = s_person_br_filt[p][ri];
                s_scratch_hr[i] = s_person_hr_filt[p][ri];
            }
            float br = estimate_bpm(s_scratch_br, buf_len, sample_rate);
            float hr = estimate_bpm(s_scratch_hr, buf_len, sample_rate);

            if (br >= 6.0f && br <= 40.0f) pv->breathing_bpm = br;
            if (br >= 6.0f && br <= 40.0f) s_person_breath_ref[p] = (s_person_breath_ref[p] > 0.0f) ? (0.85f * s_person_breath_ref[p] + 0.15f * br) : br;
            if (hr >= 40.0f && hr <= 180.0f && !hr_is_breath_harmonic(hr, s_person_breath_ref[p])) pv->heartrate_bpm = hr;
            else if (hr_is_breath_harmonic(hr, s_person_breath_ref[p])) pv->heartrate_bpm = 0.0f;
        }
    }
    for (uint8_t p = n_persons; p < BWAVE_MAX_PERSONS; p++)
        s_persons[p].active = false;
}

/* Main DSP pipeline */
static void process_frame(const dsp_ring_slot_t *slot)
{
    uint16_t n_sub = slot->iq_len / 2;
    if (n_sub == 0 || n_sub > DSP_MAX_SUBCARRIERS) return;

    s_frame_count++;
    s_latest_rssi = slot->rssi;
    const float sample_rate = 20.0f;

    /* Phase extraction + unwrapping */
    float phases[DSP_MAX_SUBCARRIERS];
    for (uint16_t sc = 0; sc < n_sub; sc++) {
        float raw = extract_phase(slot->iq_data, sc);
        phases[sc] = s_phase_init ? unwrap_phase(s_prev_phase[sc], raw) : raw;
        s_prev_phase[sc] = phases[sc];
    }
    s_phase_init = true;

    /* Welford variance */
    for (uint16_t sc = 0; sc < n_sub; sc++)
        welford_update(&s_sc_var[sc], (double)phases[sc]);

    /* Amplitude baseline EMA */
    update_amp_baseline(slot->iq_data, n_sub);

    if ((s_frame_count % 10) == 1 || s_top_k_count == 0)
        update_top_k(n_sub);

    if (s_top_k_count == 0) return;

    /* Doppler: phase rate per Top-K subcarrier */
    if (s_topk_phase_init && sample_rate > 1.0f) {
        uint8_t k = s_top_k_count < DSP_TOP_K ? s_top_k_count : DSP_TOP_K;
        for (uint8_t i = 0; i < k; i++) {
            float cur = phases[s_top_k[i]];
            float dphi = cur - s_prev_topk_phase[i];
            if (dphi > M_PI)       dphi -= 2.0f * M_PI;
            else if (dphi < -M_PI) dphi += 2.0f * M_PI;
            s_topk_velocity[i] = dphi * sample_rate;
        }
    }
    {
        uint8_t k = s_top_k_count < DSP_TOP_K ? s_top_k_count : DSP_TOP_K;
        for (uint8_t i = 0; i < k; i++)
            s_prev_topk_phase[i] = phases[s_top_k[i]];
        s_topk_phase_init = true;
    }

    float primary = phases[s_top_k[0]];

    s_phase_history[s_hist_idx] = primary;
    s_hist_idx = (s_hist_idx + 1) % DSP_PHASE_HISTORY;
    if (s_hist_len < DSP_PHASE_HISTORY) s_hist_len++;

    /* Bandpass filtering */
    float br_val = biquad_process(&s_bq_br, primary);
    float hr_val = biquad_process(&s_bq_hr, primary);

    /* Brainwave band energy */
    float dv = biquad_process(&s_bq_delta, primary);
    float tv = biquad_process(&s_bq_theta, primary);
    float av = biquad_process(&s_bq_alpha, primary);
    s_delta_energy = SMOOTH * s_delta_energy + (1.0f - SMOOTH) * (dv * dv);
    s_theta_energy = SMOOTH * s_theta_energy + (1.0f - SMOOTH) * (tv * tv);
    s_alpha_energy = SMOOTH * s_alpha_energy + (1.0f - SMOOTH) * (av * av);

    uint16_t filt_idx = (s_hist_idx + DSP_PHASE_HISTORY - 1) % DSP_PHASE_HISTORY;
    s_br_filt[filt_idx] = br_val;
    s_hr_filt[filt_idx] = hr_val;

    /* BPM estimation */
    if (s_hist_len >= 64) {
        uint16_t buf_len = s_hist_len;
        for (uint16_t i = 0; i < buf_len; i++) {
            uint16_t ri = (s_hist_idx + DSP_PHASE_HISTORY - buf_len + i) % DSP_PHASE_HISTORY;
            s_scratch_br[i] = s_br_filt[ri];
            s_scratch_hr[i] = s_hr_filt[ri];
        }

        float br = estimate_bpm(s_scratch_br, buf_len, sample_rate);
        float hr = estimate_bpm(s_scratch_hr, buf_len, sample_rate);

        if (br >= 6.0f && br <= 40.0f) {
            s_breathing_bpm = br;
            s_breath_ref = (s_breath_ref > 0.0f) ? (0.85f * s_breath_ref + 0.15f * br) : br;
        }
        if (hr >= 40.0f && hr <= 180.0f && !hr_is_breath_harmonic(hr, s_breath_ref))
            s_heartrate_bpm = hr;
        else if (hr_is_breath_harmonic(hr, s_breath_ref))
            s_heartrate_bpm = 0.0f;
    }

    /* Motion energy */
    if (s_hist_len >= 10) {
        float sum = 0.0f, sum2 = 0.0f;
        uint16_t window = (s_hist_len < 20) ? s_hist_len : 20;
        for (uint16_t i = 0; i < window; i++) {
            uint16_t ri = (s_hist_idx + DSP_PHASE_HISTORY - window + i) % DSP_PHASE_HISTORY;
            float v = s_phase_history[ri];
            sum += v; sum2 += v * v;
        }
        float mean = sum / (float)window;
        s_motion_energy = (sum2 / (float)window) - (mean * mean);
        if (s_motion_energy < 0.0f) s_motion_energy = 0.0f;
    }

    s_presence_score = s_motion_energy;
    float thresh = s_cfg.presence_thresh > 0.0f ? s_cfg.presence_thresh : 0.05f;
    s_presence_detected = (s_presence_score > thresh);

    /* Fall detection */
    if (s_hist_len >= 3) {
        uint16_t i0 = (s_hist_idx + DSP_PHASE_HISTORY - 1) % DSP_PHASE_HISTORY;
        uint16_t i1 = (s_hist_idx + DSP_PHASE_HISTORY - 2) % DSP_PHASE_HISTORY;
        float velocity = s_phase_history[i0] - s_phase_history[i1];
        float accel = fabsf(velocity - s_prev_phase_velocity);
        s_prev_phase_velocity = velocity;

        float ft = s_cfg.fall_thresh > 0.0f ? s_cfg.fall_thresh : 2.0f;
        if (accel > ft) {
            s_fall_consec_count++;
        } else {
            s_fall_consec_count = 0;
        }

        int64_t fall_now = esp_timer_get_time();
        if (s_fall_consec_count >= FALL_CONSEC_MIN
            && (fall_now - s_fall_last_alert_us) >= (int64_t)FALL_COOLDOWN_MS * 1000) {
            s_fall_detected = true;
            s_fall_last_alert_us = fall_now;
            s_fall_consec_count = 0;
            ESP_LOGW(TAG, "Fall detected! accel=%.4f", accel);
        } else if (s_fall_consec_count == 0) {
            s_fall_detected = false;
        }
    }

    /* Multi-person vitals */
    update_multi_person(slot->iq_data, n_sub, sample_rate);

    /* Send vitals at interval */
    int64_t now_us = esp_timer_get_time();
    int64_t interval_us = (int64_t)s_cfg.vital_interval_ms * 1000;
    if ((now_us - s_last_vitals_send_us) >= interval_us) {
        send_vitals();
        send_brainwave();
        send_feature_vector();
        send_person_vitals();
        send_doppler();
        send_amp_baseline(n_sub);
        s_last_vitals_send_us = now_us;

        if ((s_frame_count % 200) == 0) {
            ESP_LOGI(TAG, "br=%.1f hr=%.1f d=%.4f t=%.4f a=%.4f motion=%.4f fall=%s",
                     s_breathing_bpm, s_heartrate_bpm,
                     s_delta_energy, s_theta_energy, s_alpha_energy,
                     s_motion_energy, s_fall_detected ? "YES" : "no");
        }
    }
}

/* DSP task */
static void dsp_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "DSP task started (tier=%u)", s_cfg.tier);

    dsp_ring_slot_t slot;

    while (1) {
        uint8_t processed = 0;
        while (processed < 4 && ring_pop(&slot)) {
            process_frame(&slot);
            processed++;
        }

        if (processed == 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
        } else {
            vTaskDelay(1);
        }
    }
}

/* Public API */
bool bwave_dsp_enqueue(const uint8_t *iq, uint16_t len, int8_t rssi, uint8_t ch)
{
    return ring_push(iq, len, rssi, ch);
}

bool bwave_dsp_get_vitals(bwave_vitals_pkt_t *pkt)
{
    if (!s_pkt_valid) return false;
    memcpy(pkt, (const void *)&s_latest_pkt, sizeof(bwave_vitals_pkt_t));
    return true;
}

void bwave_dsp_get_brainwave(float *delta, float *theta, float *alpha)
{
    if (delta) *delta = s_delta_energy;
    if (theta) *theta = s_theta_energy;
    if (alpha) *alpha = s_alpha_energy;
}

void bwave_dsp_get_bpm(float *br, float *hr)
{
    if (br) *br = s_breathing_bpm;
    if (hr) *hr = s_heartrate_bpm;
}

float bwave_dsp_get_motion(void)
{
    return s_motion_energy;
}

void bwave_dsp_get_doppler(float *velocities, int *count)
{
    if (velocities) memcpy(velocities, s_topk_velocity, sizeof(float) * DSP_TOP_K);
    if (count) *count = DSP_TOP_K;
}

int bwave_dsp_get_persons(bwave_person_pkt_t *pkt)
{
    if (!pkt) return 0;
    pkt->magic = BWAVE_PERSON_MAGIC;
    pkt->node_id = 0;
    pkt->n_persons = 0;
    pkt->timestamp_ms = 0;
    for (int i = 0; i < BWAVE_MAX_PERSONS; i++) {
        pkt->persons[i].active = s_persons[i].active ? 1 : 0;
        if (s_persons[i].active) {
            pkt->persons[i].breathing_rate = (uint16_t)(s_persons[i].breathing_bpm * 100.0f);
            pkt->persons[i].heartrate = (uint16_t)(s_persons[i].heartrate_bpm * 100.0f);
            pkt->persons[i].subcarrier_idx = s_persons[i].subcarrier_idx;
            pkt->n_persons++;
        } else {
            pkt->persons[i].breathing_rate = 0;
            pkt->persons[i].heartrate = 0;
            pkt->persons[i].subcarrier_idx = 0;
        }
    }
    return pkt->n_persons;
}

void bwave_dsp_get_features(float *features, int *count)
{
    if (!features || !count) return;
    *count = 8;
    float p = s_presence_score;
    float m = s_motion_energy;
    features[0] = p > 10.0f ? 1.0f : (p < 0.0f ? 0.0f : p / 10.0f);
    features[1] = m > 10.0f ? 1.0f : (m < 0.0f ? 0.0f : m / 10.0f);
    features[2] = s_breathing_bpm > 0.0f
        ? (s_breathing_bpm / 30.0f > 1.0f ? 1.0f : s_breathing_bpm / 30.0f) : 0.0f;
    features[3] = s_heartrate_bpm > 0.0f
        ? (s_heartrate_bpm / 120.0f > 1.0f ? 1.0f : s_heartrate_bpm / 120.0f) : 0.0f;
    float var_mean = 0.0f;
    if (s_top_k_count > 0) {
        float var_sum = 0.0f;
        uint8_t k = s_top_k_count < DSP_TOP_K ? s_top_k_count : DSP_TOP_K;
        for (uint8_t i = 0; i < k; i++)
            var_sum += (float)welford_var(&s_sc_var[s_top_k[i]]);
        var_mean = var_sum / (float)k;
    }
    features[4] = var_mean > 1.0f ? 1.0f : (var_mean < 0.0f ? 0.0f : var_mean);
    int n_active = 0;
    for (int i = 0; i < BWAVE_MAX_PERSONS; i++)
        if (s_persons[i].active) n_active++;
    features[5] = (float)n_active / 4.0f;
    if (features[5] > 1.0f) features[5] = 1.0f;
    features[6] = s_fall_detected ? 1.0f : 0.0f;
    features[7] = ((float)s_latest_rssi + 100.0f) / 100.0f;
    if (features[7] > 1.0f) features[7] = 1.0f;
    if (features[7] < 0.0f) features[7] = 0.0f;
}

bool bwave_dsp_get_fall(void)
{
    return s_fall_detected;
}

float bwave_dsp_get_presence(void)
{
    return s_presence_score;
}

void bwave_dsp_get_baseline(float *baseline, int *count)
{
    if (baseline) memcpy(baseline, s_amp_baseline, sizeof(float) * BWAVE_BASELINE_MAX_SC);
    if (count) *count = s_amp_baseline_valid ? BWAVE_BASELINE_MAX_SC : 0;
}

void bwave_dsp_init_temperature(void)
{
#ifdef CONFIG_SOC_TEMP_SENSOR_SUPPORTED
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    if (temperature_sensor_install(&cfg, &s_temp_handle) == ESP_OK) {
        temperature_sensor_enable(s_temp_handle);
        temperature_sensor_get_celsius(s_temp_handle, &s_die_temp_c);
        ESP_LOGI(TAG, "Die temp: %.1f C", s_die_temp_c);
    }
#endif
}

esp_err_t bwave_dsp_init(const bwave_dsp_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;

    s_cfg = *cfg;
    memset(&s_ring, 0, sizeof(s_ring));
    memset(s_sc_var, 0, sizeof(s_sc_var));
    memset(s_prev_phase, 0, sizeof(s_prev_phase));
    s_phase_init = false;
    s_top_k_count = 0;
    s_hist_len = s_hist_idx = 0;
    s_breathing_bpm = s_heartrate_bpm = 0.0f;
    s_motion_energy = s_presence_score = 0.0f;
    s_presence_detected = false;
    s_frame_count = 0;
    s_breath_ref = 0.0f;
    s_last_vitals_send_us = 0;
    s_pkt_valid = false;
    s_delta_energy = s_theta_energy = s_alpha_energy = 0.0f;

    memset(s_topk_velocity, 0, sizeof(s_topk_velocity));
    memset(s_prev_topk_phase, 0, sizeof(s_prev_topk_phase));
    s_topk_phase_init = false;
    memset(s_persons, 0, sizeof(s_persons));
    memset(s_person_breath_ref, 0, sizeof(s_person_breath_ref));
    s_feature_seq = 0;
    s_fall_detected = false;
    s_prev_phase_velocity = 0.0f;
    s_fall_consec_count = 0;
    s_fall_last_alert_us = 0;
    memset(s_amp_baseline, 0, sizeof(s_amp_baseline));
    s_amp_baseline_valid = false;
    s_last_baseline_send_us = 0;

    const float fs = 20.0f;
    biquad_design(&s_bq_br, fs, 0.1f, 0.5f);
    biquad_design(&s_bq_hr, fs, 0.8f, 2.0f);
    biquad_design(&s_bq_delta, fs, 0.5f, 4.0f);
    biquad_design(&s_bq_theta, fs, 4.0f, 8.0f);
    biquad_design(&s_bq_alpha, fs, 8.0f, 9.5f);
    for (uint8_t p = 0; p < BWAVE_MAX_PERSONS; p++) {
        biquad_design(&s_person_bq_br[p], fs, 0.1f, 0.5f);
        biquad_design(&s_person_bq_hr[p], fs, 0.8f, 2.0f);
    }

    if (s_cfg.tier == 0) {
        ESP_LOGI(TAG, "Tier 0: raw passthrough");
        return ESP_OK;
    }

    bwave_dsp_init_temperature();

    BaseType_t ret = xTaskCreate(dsp_task, "bwave_dsp", 8192, NULL, 5, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create DSP task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "DSP initialized (tier=%u, top_k=%u, interval=%ums)",
             s_cfg.tier, s_cfg.top_k_count, s_cfg.vital_interval_ms);
    return ESP_OK;
}
