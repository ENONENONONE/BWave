#ifndef BWAVE_DSP_H
#define BWAVE_DSP_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

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

typedef struct {
    uint8_t  tier;
    float    presence_thresh;
    float    fall_thresh;
    uint16_t vital_interval_ms;
    uint8_t  top_k_count;
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

#endif
