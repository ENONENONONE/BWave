#include "bwave_resonance.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define HANN_BW_BINS 1.44f

void bwave_res_prepare(float *x, uint16_t n)
{
    if (!x || n < 2) return;

    /* least-squares line a + b*i */
    float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        float fi = (float)i;
        sx += fi; sy += x[i]; sxx += fi * fi; sxy += fi * x[i];
    }
    float fn = (float)n;
    float den = fn * sxx - sx * sx;
    float b = (den != 0.0f) ? (fn * sxy - sx * sy) / den : 0.0f;
    float a = (sy - b * sx) / fn;

    for (uint16_t i = 0; i < n; i++) {
        float w = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * (float)i / (float)(n - 1));
        x[i] = (x[i] - a - b * (float)i) * w;
    }
}

float bwave_res_power(const float *x, uint16_t n, float fs, float f_hz)
{
    float coeff = 2.0f * cosf(2.0f * (float)M_PI * f_hz / fs);
    float s1 = 0.0f, s2 = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        float s = x[i] + coeff * s1 - s2;
        s2 = s1;
        s1 = s;
    }
    float p = s1 * s1 + s2 * s2 - coeff * s1 * s2;
    return p > 0.0f ? p : 0.0f;
}

float bwave_res_bw_floor(uint16_t n, float fs)
{
    return (n > 0) ? HANN_BW_BINS * fs / (float)n : 0.0f;
}

/* Frequency in (f_in, f_out) where power crosses target, P(f_in) >= target > P(f_out). */
static float bisect_half_power(const float *x, uint16_t n, float fs,
                               float f_in, float f_out, float target)
{
    for (int it = 0; it < 12; it++) {
        float fm = 0.5f * (f_in + f_out);
        if (bwave_res_power(x, n, fs, fm) >= target) f_in = fm;
        else                                         f_out = fm;
    }
    return 0.5f * (f_in + f_out);
}

/* scratch for bwave_res_peak (DSP task only; not reentrant) */
static float s_pw[BWAVE_RES_SCAN_MAX];
static float s_sorted[BWAVE_RES_SCAN_MAX];

static int cmp_float(const void *a, const void *b)
{
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

bool bwave_res_peak(const float *x, uint16_t n, float fs,
                    float scan_lo, float scan_hi,
                    float accept_lo, float accept_hi,
                    bwave_res_peak_t *out)
{
    if (!out) return false;
    memset(out, 0, sizeof(*out));
    if (!x || n < 16 || fs <= 0.0f || scan_hi <= scan_lo) return false;
    if (scan_hi > 0.5f * fs) scan_hi = 0.5f * fs;

    /* coarse grid no wider than half the window's resolution so a sharp
       peak cannot fall between points */
    float floor_bw = bwave_res_bw_floor(n, fs);
    int np = BWAVE_RES_SCAN_POINTS;
    int need = (int)((scan_hi - scan_lo) / (0.5f * floor_bw)) + 1;
    if (need > np) np = need;
    if (np > BWAVE_RES_SCAN_MAX) np = BWAVE_RES_SCAN_MAX;
    float step = (scan_hi - scan_lo) / (float)(np - 1);

    float *pw = s_pw;
    int kmax = -1;
    for (int k = 0; k < np; k++) {
        float f = scan_lo + step * (float)k;
        pw[k] = bwave_res_power(x, n, fs, f);
        if (f >= accept_lo && f <= accept_hi && (kmax < 0 || pw[k] > pw[kmax]))
            kmax = k;
    }
    if (kmax < 0 || pw[kmax] <= 0.0f) return false;

    memcpy(s_sorted, pw, sizeof(float) * (size_t)np);
    qsort(s_sorted, (size_t)np, sizeof(float), cmp_float);
    float median = s_sorted[np / 2];
    out->snr = (median > 0.0f) ? pw[kmax] / median : 1e6f;

    bool interior = (kmax > 0 && kmax < np - 1);
    if (!interior) return false;

    /* refine f0: golden-section search between the neighbouring points */
    float a = scan_lo + step * (float)(kmax - 1);
    float b = scan_lo + step * (float)(kmax + 1);
    const float gr = 0.618034f;
    float c = b - gr * (b - a), d = a + gr * (b - a);
    float pc = bwave_res_power(x, n, fs, c), pd = bwave_res_power(x, n, fs, d);
    for (int it = 0; it < 16; it++) {
        if (pc > pd) { b = d; d = c; pd = pc; c = b - gr * (b - a); pc = bwave_res_power(x, n, fs, c); }
        else         { a = c; c = d; pc = pd; d = a + gr * (b - a); pd = bwave_res_power(x, n, fs, d); }
    }
    float f0 = 0.5f * (a + b);
    float pmax = bwave_res_power(x, n, fs, f0);
    if (pw[kmax] > pmax) { f0 = scan_lo + step * (float)kmax; pmax = pw[kmax]; }
    out->f0_hz = f0;

    /* half-power points: step outward (growing) until below half, then bisect */
    float half = 0.5f * pmax;
    bool have_lo = false, have_hi = false;
    for (int side = -1; side <= 1; side += 2) {
        float dd = 0.25f * floor_bw, prev = 0.0f;
        for (int it = 0; it < 64; it++) {
            float f = f0 + (float)side * dd;
            if (f < scan_lo || f > scan_hi) break;
            if (bwave_res_power(x, n, fs, f) < half) {
                float fe = bisect_half_power(x, n, fs, f0 + (float)side * prev, f, half);
                if (side < 0) { out->f1_hz = fe; have_lo = true; }
                else          { out->f2_hz = fe; have_hi = true; }
                break;
            }
            prev = dd;
            dd *= 1.5f;
        }
    }

    bool in_accept = (f0 >= accept_lo && f0 <= accept_hi);
    if (have_lo && have_hi && out->f2_hz > out->f1_hz) {
        float bw = out->f2_hz - out->f1_hz;
        out->q = f0 / bw;
        out->res_limited = (bw < 1.25f * floor_bw);
    }
    out->valid = in_accept && out->q > 0.0f
              && out->snr >= BWAVE_RES_MIN_SNR;
    return out->valid;
}

float bwave_res_log_decrement(const float *x, uint16_t n, uint8_t *n_cycles)
{
    if (n_cycles) *n_cycles = 0;
    if (!x || n < 4) return 0.0f;

    /* one peak per cycle: max between successive upward zero crossings */
    float sk = 0.0f, sl = 0.0f, skk = 0.0f, skl = 0.0f;
    int k = 0;
    bool in_cycle = false;
    float cyc_max = 0.0f;
    for (uint16_t i = 1; i < n; i++) {
        if (x[i - 1] <= 0.0f && x[i] > 0.0f) {
            if (in_cycle && cyc_max > 0.0f) {
                float l = logf(cyc_max);
                sk += (float)k; sl += l; skk += (float)k * (float)k; skl += (float)k * l;
                k++;
                if (k == 255) break;
            }
            in_cycle = true;
            cyc_max = 0.0f;
        }
        if (in_cycle && x[i] > cyc_max) cyc_max = x[i];
    }
    if (n_cycles) *n_cycles = (uint8_t)k;
    if (k < 2) return 0.0f;

    float fk = (float)k;
    float den = fk * skk - sk * sk;
    if (den == 0.0f) return 0.0f;
    float slope = (fk * skl - sk * sl) / den;
    return -slope;
}

float bwave_res_sine_peak(const float *x, uint16_t n)
{
    if (!x || n == 0) return 0.0f;
    float s2 = 0.0f;
    for (uint16_t i = 0; i < n; i++) s2 += x[i] * x[i];
    return sqrtf(2.0f * s2 / (float)n);
}

void bwave_res_bandpass(float fs, float f0, float q, bwave_res_bq_coef_t *out)
{
    if (!out) return;
    if (q <= 0.0f) q = 0.5f;
    float K = tanf((float)M_PI * f0 / fs);
    float kq = K / q;
    float a0 = K * K + kq + 1.0f;
    out->b0 =  kq / a0;
    out->b1 =  0.0f;
    out->b2 = -kq / a0;
    out->a1 = 2.0f * (K * K - 1.0f) / a0;
    out->a2 = (K * K - kq + 1.0f) / a0;
}

float bwave_res_band_q(float f_lo, float f_hi, float *f0_out)
{
    float f0 = sqrtf(f_lo * f_hi);
    if (f0_out) *f0_out = f0;
    return (f_hi > f_lo) ? f0 / (f_hi - f_lo) : 0.0f;
}

float bwave_res_selectivity(float f, float f0, float q, uint8_t n_stages)
{
    if (f <= 0.0f || f0 <= 0.0f) return 0.0f;
    if (n_stages < 1) n_stages = 1;
    float y = f / f0 - f0 / f;
    float qy = q * y;
    return powf(1.0f + qy * qy, -0.5f * (float)n_stages);
}

void bwave_res_band_edges(float f0, float q, uint8_t n_stages,
                          float *f1, float *f2)
{
    if (n_stages < 1) n_stages = 1;
    float y = (q > 0.0f) ? sqrtf(powf(2.0f, 1.0f / (float)n_stages) - 1.0f) / q : 0.0f;
    /* f/f0 - f0/f = +-y  ->  f = f0 (sqrt(1 + y^2/4) +- y/2) */
    float r = sqrtf(1.0f + 0.25f * y * y);
    if (f1) *f1 = f0 * (r - 0.5f * y);
    if (f2) *f2 = f0 * (r + 0.5f * y);
}

float bwave_res_q_from_decrement(float delta)
{
    /* delta = r / (2 fn L), Q = w0 L / r, fn = f0 sqrt(1 - 1/(4 Q^2))
       -> Q^2 = pi^2 / delta^2 + 1/4 (exact; Q ~= pi / delta for small delta) */
    if (delta <= 0.0f) return 0.0f;
    return sqrtf((float)(M_PI * M_PI) / (delta * delta) + 0.25f);
}
