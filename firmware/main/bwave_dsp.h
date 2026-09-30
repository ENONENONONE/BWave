#ifndef BWAVE_DSP_H
#define BWAVE_DSP_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "bwave_resonance.h"

#define DSP_RING_SLOTS       16
#define DSP_MAX_IQ_BYTES     1024
#define DSP_PHASE_HISTORY    256
#define DSP_TOP_K            8
#define DSP_MAX_SUBCARRIERS  256

typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float x1, x2;
    float y1, y2;
} bwave_biquad_t;

#define BWAVE_VITALS_MAGIC   0xC5110002
#define BWAVE_FEATURE_MAGIC  0xC5110003
#define BWAVE_BWAVE_MAGIC    0xC5110009
#define BWAVE_PERSON_MAGIC   0xC511000A
#define BWAVE_DOPPLER_MAGIC  0xC511000E
#define BWAVE_BASELINE_MAGIC 0xC5110008
#define BWAVE_RESONANCE_MAGIC 0xC511000F

#define BWAVE_MAX_PERSONS    4
#define BWAVE_BASELINE_MAX_SC 256

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t  node_id;
    uint8_t  flags;
    uint16_t breathing_rate;
    uint32_t heartrate;
    int8_t   rssi;
    uint8_t  n_persons;
    int8_t   die_temp_c;
    uint8_t  reserved1;
    float    motion_energy;
    float    presence_score;
    uint32_t timestamp_ms;
    uint32_t reserved2;
} bwave_vitals_pkt_t;

_Static_assert(sizeof(bwave_vitals_pkt_t) == 32, "vitals packet must be 32 bytes");

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t  node_id;
    uint8_t  reserved;
    uint16_t timestamp_ms;
    float    delta;
    float    theta;
    float    alpha;
    uint32_t reserved2;
} bwave_band_pkt_t;

_Static_assert(sizeof(bwave_band_pkt_t) == 24, "band packet must be 24 bytes");

/* Feature vector packet (48 bytes) */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t  node_id;
    uint8_t  reserved;
    uint16_t seq;
    int64_t  timestamp_us;
    float    features[8];
} bwave_feature_pkt_t;

_Static_assert(sizeof(bwave_feature_pkt_t) == 48, "feature packet must be 48 bytes");

/* Per-person vitals */
typedef struct __attribute__((packed)) {
    uint16_t breathing_rate;
    uint16_t heartrate;
    uint8_t  subcarrier_idx;
    uint8_t  active;
} bwave_person_entry_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t  node_id;
    uint8_t  n_persons;
    uint16_t timestamp_ms;
    bwave_person_entry_t persons[BWAVE_MAX_PERSONS];
} bwave_person_pkt_t;

/* Doppler velocity packet (40 bytes) */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t  node_id;
    uint8_t  n_carriers;
    uint16_t timestamp_ms;
    float    velocity[DSP_TOP_K];
} bwave_doppler_pkt_t;

_Static_assert(sizeof(bwave_doppler_pkt_t) == 40, "doppler packet must be 40 bytes");

/* Resonance packet (48 bytes), sent about once a second at tier 2.
 * flags: bit0 br valid, bit1 hr valid, bit2 am valid,
 *        bit3 br Q resolution-limited, bit4 hr Q resolution-limited,
 *        bit5 AM and PM breathing agree, bit6 am Q resolution-limited */
#define BWAVE_RES_F_BR_VALID   0x01
#define BWAVE_RES_F_HR_VALID   0x02
#define BWAVE_RES_F_AM_VALID   0x04
#define BWAVE_RES_F_BR_RESLIM  0x08
#define BWAVE_RES_F_HR_RESLIM  0x10
#define BWAVE_RES_F_AGREE      0x20
#define BWAVE_RES_F_AM_RESLIM  0x40

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t  node_id;
    uint8_t  flags;
    uint16_t timestamp_ms;
    float    br_f0_hz;      /* phase (PM) breathing peak */
    float    br_q;          /* f0 / half-power bandwidth */
    float    hr_f0_hz;
    float    hr_q;
    float    br_decrement;  /* log decrement per cycle, breathing band */
    float    hr_decrement;
    float    am_depth;      /* AM modulation depth m, breathing band */
    float    pm_index;      /* peak phase deviation (rad), breathing band */
    float    am_br_f0_hz;   /* amplitude (AM) breathing peak */
    float    am_br_q;
} bwave_resonance_pkt_t;

_Static_assert(sizeof(bwave_resonance_pkt_t) == 48, "resonance packet must be 48 bytes");

typedef struct {
    bwave_res_peak_t br;      /* from phase */
    bwave_res_peak_t hr;      /* from phase */
    bwave_res_peak_t am_br;   /* from amplitude */
    float    br_decrement;
    float    hr_decrement;
    uint8_t  br_cycles;
    uint8_t  hr_cycles;
    float    am_depth;
    float    pm_index;
    bool     am_pm_agree;
    uint16_t n_samples;
    uint32_t updated_ms;
} bwave_dsp_resonance_t;

/* Vital-sign band-pass filters: n identical stages at centre f0, per-stage Q */
typedef struct {
    float   br_f0, br_q;
    uint8_t br_stages;
    float   hr_f0, hr_q;
    uint8_t hr_stages;
} bwave_dsp_filter_t;

typedef struct {
    uint8_t  tier;
    float    presence_thresh;
    float    fall_thresh;
    uint16_t vital_interval_ms;
    uint8_t  top_k_count;
    float    br_q;        /* 0 = Q of the 0.1-0.5 Hz band */
    float    hr_q;        /* 0 = Q of the 0.8-2.0 Hz band */
    uint8_t  br_stages;   /* 1..4, 0 = 1 */
    uint8_t  hr_stages;
} bwave_dsp_config_t;

esp_err_t bwave_dsp_init(const bwave_dsp_config_t *cfg);

bool bwave_dsp_enqueue(const uint8_t *iq_data, uint16_t iq_len,
                       int8_t rssi, uint8_t channel);

bool bwave_dsp_get_vitals(bwave_vitals_pkt_t *pkt);

void bwave_dsp_get_brainwave(float *delta, float *theta, float *alpha);

void bwave_dsp_get_bpm(float *br, float *hr);

float bwave_dsp_get_motion(void);

void  bwave_dsp_get_doppler(float *velocities, int *count);
int   bwave_dsp_get_persons(bwave_person_pkt_t *pkt);
void  bwave_dsp_get_features(float *features, int *count);
bool  bwave_dsp_get_fall(void);
float bwave_dsp_get_presence(void);
void  bwave_dsp_get_baseline(float *baseline, int *count);

/* false until the first resonance measurement */
bool  bwave_dsp_get_resonance(bwave_dsp_resonance_t *out);
void  bwave_dsp_get_filter(bwave_dsp_filter_t *out);
/* Applied by the DSP task on its next frame. q < 0 or stages 0 keeps the
 * current value; q == 0 restores the band default. */
void  bwave_dsp_set_filter(float br_q, float hr_q,
                           uint8_t br_stages, uint8_t hr_stages);

#endif
