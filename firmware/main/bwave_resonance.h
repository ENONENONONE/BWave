#ifndef BWAVE_RESONANCE_H
#define BWAVE_RESONANCE_H

/* Tuned-circuit measurements applied to CSI time series.
 *
 * Pure C + libm, no ESP-IDF dependency, so it can be unit tested on the
 * host (firmware/test/host). Notation follows the Radiotron Designer's
 * Handbook ch. 9: f0 resonant frequency, Q magnification factor,
 * Y = f/f0 - f0/f, delta logarithmic decrement.
 */

#include <stdint.h>
#include <stdbool.h>

#define BWAVE_RES_SCAN_POINTS 48
#define BWAVE_RES_SCAN_MAX    256
#define BWAVE_RES_MIN_SNR     10.0f  /* ~1.5% false peaks on white noise, 256 samples */
#define BWAVE_RES_MAX_STAGES  4

/* Spectral peak measured by half-power points (eq. 29-31):
 * Q = f0 / (f2 - f1), f1/f2 where power is 1/2 (amplitude 70.7%) of peak. */
typedef struct {
    float f0_hz;
    float f1_hz;
    float f2_hz;
    float q;
    float snr;          /* peak power / median power over the scan */
    bool  valid;        /* peak inside accept range, both f1 and f2 found, snr ok */
    bool  res_limited;  /* f2 - f1 is at the window's resolution floor: Q is a lower bound */
} bwave_res_peak_t;

/* Band-pass biquad coefficients (a0 normalised to 1). */
typedef struct {
    float b0, b1, b2;
    float a1, a2;
} bwave_res_bq_coef_t;

/* In place: remove least-squares line (phase drift) and apply Hann window. */
void bwave_res_prepare(float *x, uint16_t n);

/* Power of prepared series x at frequency f (Goertzel). */
float bwave_res_power(const float *x, uint16_t n, float fs, float f_hz);

/* Not reentrant (uses static scratch); call from one task only.
 * Find the strongest peak in [scan_lo, scan_hi]; it is valid only if its
 * centre lies in [accept_lo, accept_hi]. x must already be prepared.
 * Returns out->valid. */
bool bwave_res_peak(const float *x, uint16_t n, float fs,
                    float scan_lo, float scan_hi,
                    float accept_lo, float accept_hi,
                    bwave_res_peak_t *out);

/* -3 dB bandwidth floor of an n-sample Hann window (1.44 bins). */
float bwave_res_bw_floor(uint16_t n, float fs);

/* Logarithmic decrement, eq. 5: delta = ln(i / i'), ratio of successive
 * cycle amplitudes. Least-squares slope of ln(peak) over all cycles in x,
 * so delta > 0 decaying, ~0 sustained, < 0 growing. Q ~= pi / delta.
 * With two cycles this is exactly ln(i/i'). x should be band-limited to
 * one rhythm. Returns 0 and *n_cycles < 2 when there are too few cycles. */
float bwave_res_log_decrement(const float *x, uint16_t n, uint8_t *n_cycles);

/* Exact Q of the damped circuit that decays by delta per cycle
 * (sect. 11 eqs. 1, 9): Q = sqrt(pi^2/delta^2 + 1/4). 0 if delta <= 0. */
float bwave_res_q_from_decrement(float delta);

/* sqrt(2) * rms(x): peak of a sinusoid with the same power. */
float bwave_res_sine_peak(const float *x, uint16_t n);

/* Band-pass biquad centred on f0 with magnification Q (0 dB at f0).
 * Analogue prototype H(s) = (s/(Q w0)) / (1 + s/(Q w0) + (s/w0)^2),
 * bilinear transform with f0 prewarped. */
void bwave_res_bandpass(float fs, float f0, float q, bwave_res_bq_coef_t *out);

/* Q whose half-power points are exactly f_lo and f_hi: f0 = sqrt(f_lo f_hi),
 * Q = f0 / (f_hi - f_lo). */
float bwave_res_band_q(float f_lo, float f_hi, float *f0_out);

/* Selectivity of n identical single tuned stages in cascade
 * (ch. 9 sect. 6 eq. 24 and sect. 9): A/A0 = (1 + Q^2 Y^2)^(-n/2). */
float bwave_res_selectivity(float f, float f0, float q, uint8_t n_stages);

/* Overall half-power points of n identical stages: solves
 * (1 + Q^2 Y^2)^n = 2, i.e. Q Y = +-sqrt(2^(1/n) - 1). */
void bwave_res_band_edges(float f0, float q, uint8_t n_stages,
                          float *f1, float *f2);

#endif
