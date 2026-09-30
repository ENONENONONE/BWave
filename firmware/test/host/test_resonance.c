/* Host tests for bwave_resonance.c. Run: firmware/test/host/run.sh */

#include "bwave_resonance.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int s_fail, s_pass;

#define CHECK(cond, ...) do { \
    if (cond) { s_pass++; } \
    else { s_fail++; printf("FAIL %s:%d: ", __FILE__, __LINE__); \
           printf(__VA_ARGS__); printf("\n"); } \
} while (0)

#define NEAR(a, b, tol) (fabsf((float)(a) - (float)(b)) <= (tol))

static float s_buf[4096];

/* Tiny LCG so runs are reproducible */
static uint32_t s_rng = 12345;
static float noise(void)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return ((float)(s_rng >> 8) / 16777216.0f) * 2.0f - 1.0f;
}

static float bq_run(const bwave_res_bq_coef_t *c, float *st, float x)
{
    /* st: x1 x2 y1 y2 */
    float y = c->b0 * x + c->b1 * st[0] + c->b2 * st[1] - c->a1 * st[2] - c->a2 * st[3];
    st[1] = st[0]; st[0] = x; st[3] = st[2]; st[2] = y;
    return y;
}

/* Steady-state magnitude of a cascade of n biquads at f (by simulation) */
static float bq_gain(const bwave_res_bq_coef_t *c, int n_stages, float fs, float f)
{
    float st[4][4] = {{0}};
    int settle = (int)(fs * 400.0f), meas = (int)(fs * 100.0f);
    float peak = 0.0f;
    for (int i = 0; i < settle + meas; i++) {
        float v = sinf(2.0f * (float)M_PI * f * (float)i / fs);
        for (int s = 0; s < n_stages; s++) v = bq_run(c, st[s], v);
        if (i >= settle && fabsf(v) > peak) peak = fabsf(v);
    }
    return peak;
}

static void test_peak_clean_sine(void)
{
    const float fs = 20.0f;
    const uint16_t n = 256;
    for (int i = 0; i < n; i++)
        s_buf[i] = sinf(2.0f * (float)M_PI * 0.25f * i / fs) + 0.01f * i; /* + drift */
    bwave_res_prepare(s_buf, n);
    bwave_res_peak_t pk;
    bool ok = bwave_res_peak(s_buf, n, fs, 0.05f, 0.8f, 0.1f, 0.667f, &pk);
    CHECK(ok, "clean breathing sine should be valid (snr=%.1f q=%.2f)", pk.snr, pk.q);
    CHECK(NEAR(pk.f0_hz, 0.25f, 0.005f), "f0=%.4f want 0.25", pk.f0_hz);
    CHECK(pk.res_limited, "12.8 s window should be flagged resolution-limited");
}

/* Clean sine over a long window: half-power width equals the Hann
 * mainlobe (1.44 bins), so Q = f0 / (1.44 fs / n). Checks the
 * half-power search itself against a known answer. */
static void test_q_matches_hann_width(void)
{
    const float fs = 20.0f;
    const uint16_t n = 2048;
    const float f = 1.0f;
    for (int i = 0; i < n; i++) s_buf[i] = sinf(2.0f * (float)M_PI * f * i / fs);
    bwave_res_prepare(s_buf, n);
    bwave_res_peak_t pk;
    bwave_res_peak(s_buf, n, fs, 0.6f, 3.2f, 0.667f, 3.0f, &pk);
    float want_q = f / bwave_res_bw_floor(n, fs);
    CHECK(pk.valid, "long hr sine should be valid");
    CHECK(NEAR(pk.q, want_q, 0.03f * want_q), "q=%.2f want %.2f", pk.q, want_q);
    CHECK(NEAR(pk.f0_hz, f, 0.002f), "f0=%.4f", pk.f0_hz);
}

/* Irregular rhythm (random-walk frequency) must read lower Q than a clean one */
static void test_q_drops_with_jitter(void)
{
    const float fs = 20.0f;
    const uint16_t n = 2048;
    float phase = 0.0f, fj = 1.2f;
    for (int i = 0; i < n; i++) s_buf[i] = sinf(2.0f * (float)M_PI * 1.2f * i / fs);
    bwave_res_prepare(s_buf, n);
    bwave_res_peak_t clean;
    bwave_res_peak(s_buf, n, fs, 0.6f, 3.2f, 0.667f, 3.0f, &clean);

    for (int i = 0; i < n; i++) {
        fj += 0.01f * noise();
        if (fj < 1.0f) fj = 1.0f;
        if (fj > 1.4f) fj = 1.4f;
        phase += 2.0f * (float)M_PI * fj / fs;
        s_buf[i] = sinf(phase);
    }
    bwave_res_prepare(s_buf, n);
    bwave_res_peak_t jit;
    bwave_res_peak(s_buf, n, fs, 0.6f, 3.2f, 0.667f, 3.0f, &jit);
    CHECK(jit.q > 0.0f && jit.q < 0.7f * clean.q,
          "jittered q=%.2f should be well below clean q=%.2f", jit.q, clean.q);
}

static void test_peak_rejects_noise_and_out_of_band(void)
{
    const float fs = 20.0f;
    const uint16_t n = 256;
    bwave_res_peak_t pk;
    int valid_noise = 0;
    for (int trial = 0; trial < 100; trial++) {
        for (int i = 0; i < n; i++) s_buf[i] = noise();
        bwave_res_prepare(s_buf, n);
        if (bwave_res_peak(s_buf, n, fs, 0.05f, 0.8f, 0.1f, 0.667f, &pk)) valid_noise++;
    }
    CHECK(valid_noise <= 5, "white noise produced %d/100 valid peaks", valid_noise);

    /* breathing buried in noise of equal amplitude must still be found */
    int found = 0;
    for (int trial = 0; trial < 20; trial++) {
        for (int i = 0; i < n; i++)
            s_buf[i] = sinf(2.0f * (float)M_PI * 0.3f * i / fs) + noise();
        bwave_res_prepare(s_buf, n);
        if (bwave_res_peak(s_buf, n, fs, 0.05f, 0.8f, 0.1f, 0.667f, &pk)
            && NEAR(pk.f0_hz, 0.3f, 0.02f)) found++;
    }
    CHECK(found >= 18, "noisy breathing found %d/20", found);

    /* 0.75 Hz tone: outside breathing accept range */
    for (int i = 0; i < n; i++) s_buf[i] = sinf(2.0f * (float)M_PI * 0.78f * i / fs);
    bwave_res_prepare(s_buf, n);
    CHECK(!bwave_res_peak(s_buf, n, fs, 0.05f, 0.8f, 0.1f, 0.667f, &pk),
          "out-of-band tone accepted as breathing (f0=%.3f)", pk.f0_hz);
}

/* Damped oscillation, eq. 1 / 4 / 5: i = I e^(-a t) cos(w t),
 * delta = a / fn per cycle */
static void test_log_decrement(void)
{
    const float fs = 20.0f;
    const uint16_t n = 256;
    const float fn = 0.5f, a = 0.1f;
    for (int i = 0; i < n; i++) {
        float t = i / fs;
        s_buf[i] = expf(-a * t) * sinf(2.0f * (float)M_PI * fn * t);
    }
    uint8_t cyc;
    float d = bwave_res_log_decrement(s_buf, n, &cyc);
    CHECK(cyc >= 5, "cycles=%u", cyc);
    CHECK(NEAR(d, a / fn, 0.01f), "delta=%.4f want %.4f", d, a / fn);

    /* eq. 9 with eq. 1: delta = pi/(Q sqrt(1 - 1/4Q^2)) inverts exactly */
    const float qs[] = { 0.8f, 2.0f, 10.0f, 50.0f };
    for (int i = 0; i < 4; i++) {
        float q = qs[i];
        float delta = (float)M_PI / (q * sqrtf(1.0f - 1.0f / (4.0f * q * q)));
        float qb = bwave_res_q_from_decrement(delta);
        CHECK(NEAR(qb, q, 1e-3f * q), "q_from_decrement(%.4f)=%.4f want %.2f", delta, qb, q);
    }
    CHECK(bwave_res_q_from_decrement(0.0f) == 0.0f, "undamped -> 0");

    for (int i = 0; i < n; i++) s_buf[i] = sinf(2.0f * (float)M_PI * fn * i / fs);
    d = bwave_res_log_decrement(s_buf, n, &cyc);
    CHECK(NEAR(d, 0.0f, 0.005f), "sustained delta=%.4f want 0", d);

    for (int i = 0; i < 40; i++) s_buf[i] = sinf(2.0f * (float)M_PI * fn * i / fs);
    d = bwave_res_log_decrement(s_buf, 40, &cyc);
    CHECK(cyc < 2 && d == 0.0f, "too few cycles should give 0 (cyc=%u d=%.3f)", cyc, d);

    /* two full cycles: 12.8 s window at 12 bpm, the breathing worst case */
    const float fb = 0.2f, ab = 0.05f;
    for (int i = 0; i < n; i++) {
        float t = i / fs;
        s_buf[i] = expf(-ab * t) * sinf(2.0f * (float)M_PI * fb * t);
    }
    d = bwave_res_log_decrement(s_buf, n, &cyc);
    CHECK(cyc == 2 && NEAR(d, ab / fb, 0.01f), "2-cycle delta=%.4f want %.4f (cyc=%u)", d, ab / fb, cyc);
}

/* Designed biquad: unity at f0, 70.7% at the eq.-29 band edges, and
 * following the analogue selectivity curve eq. 24 well below Nyquist. */
static void test_bandpass_design(void)
{
    const float fs = 20.0f;
    float f0;
    float q = bwave_res_band_q(0.8f, 2.0f, &f0);
    CHECK(NEAR(f0, sqrtf(1.6f), 1e-4f) && NEAR(q, f0 / 1.2f, 1e-4f), "band_q f0=%.4f q=%.4f", f0, q);

    bwave_res_bq_coef_t c;
    bwave_res_bandpass(fs, f0, q, &c);
    CHECK(NEAR(bq_gain(&c, 1, fs, f0), 1.0f, 0.01f), "gain at f0");

    float f1, f2;
    bwave_res_band_edges(f0, q, 1, &f1, &f2);
    CHECK(NEAR(f1, 0.8f, 1e-3f) && NEAR(f2, 2.0f, 1e-3f), "edges %.4f %.4f", f1, f2);

    /* analogue vs digital: bilinear warp is small at f << fs/2 */
    const float fq[] = { 0.3f, 0.8f, 2.0f, 3.0f };
    for (int i = 0; i < 4; i++) {
        float g = bq_gain(&c, 1, fs, fq[i]);
        float s = bwave_res_selectivity(fq[i], f0, q, 1);
        CHECK(NEAR(g, s, 0.04f), "f=%.2f digital %.4f vs eq.24 %.4f", fq[i], g, s);
    }
}

/* Sect. 9: n identical stages -> (1 + Q^2 Y^2)^(-n/2), overall band narrows */
static void test_cascade(void)
{
    const float fs = 20.0f, f0 = 0.25f, q = 2.0f;
    bwave_res_bq_coef_t c;
    bwave_res_bandpass(fs, f0, q, &c);

    float prev_bw = 1e9f;
    for (uint8_t n = 1; n <= BWAVE_RES_MAX_STAGES; n++) {
        float f1, f2;
        bwave_res_band_edges(f0, q, n, &f1, &f2);
        CHECK(NEAR(bwave_res_selectivity(f1, f0, q, n), 0.7071f, 1e-3f), "n=%u sel(f1)", n);
        CHECK(NEAR(bwave_res_selectivity(f2, f0, q, n), 0.7071f, 1e-3f), "n=%u sel(f2)", n);
        CHECK(f2 - f1 < prev_bw, "n=%u band should narrow", n);
        prev_bw = f2 - f1;

        float g = bq_gain(&c, n, fs, f2);
        CHECK(NEAR(g, 0.7071f, 0.02f), "n=%u digital cascade at f2 = %.4f", n, g);
    }
    float s1 = bwave_res_selectivity(0.5f, f0, q, 1);
    float s3 = bwave_res_selectivity(0.5f, f0, q, 3);
    CHECK(NEAR(s3, s1 * s1 * s1, 1e-5f), "cascade is product of stages");
}

static void test_sine_peak(void)
{
    const uint16_t n = 400;
    for (int i = 0; i < n; i++) s_buf[i] = 0.3f * sinf(2.0f * (float)M_PI * i / 40.0f);
    CHECK(NEAR(bwave_res_sine_peak(s_buf, n), 0.3f, 1e-3f), "sine peak");
}

/* AM depth as the firmware computes it: band-pass the amplitude, take
 * sine peak, divide by carrier mean. A(1 + m cos wt) with m = 0.2. */
static void test_am_depth(void)
{
    const float fs = 20.0f, fm = 0.25f, m = 0.2f, A = 30.0f;
    float f0;
    float q = bwave_res_band_q(0.1f, 0.5f, &f0);
    bwave_res_bq_coef_t c;
    bwave_res_bandpass(fs, f0, q, &c);
    float st[4] = {0};
    const int warm = 400, n = 256;
    float raw[256];
    for (int i = 0; i < warm + n; i++) {
        float a = A * (1.0f + m * cosf(2.0f * (float)M_PI * fm * i / fs));
        float y = bq_run(&c, st, a);
        if (i >= warm) { s_buf[i - warm] = y; raw[i - warm] = a; }
    }
    float mean = 0.0f;
    for (int i = 0; i < n; i++) mean += raw[i];
    mean /= n;
    float depth = bwave_res_sine_peak(s_buf, n) / mean;
    float g = bwave_res_selectivity(fm, f0, q, 1);
    CHECK(NEAR(depth, m * g, 0.01f), "depth=%.4f want %.4f", depth, m * g);
}

int main(void)
{
    test_peak_clean_sine();
    test_q_matches_hann_width();
    test_q_drops_with_jitter();
    test_peak_rejects_noise_and_out_of_band();
    test_log_decrement();
    test_bandpass_design();
    test_cascade();
    test_sine_peak();
    test_am_depth();
    printf("%d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
