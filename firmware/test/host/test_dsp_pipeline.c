/* Drives the real bwave_dsp.c pipeline with synthetic CSI on the host.
 * One subcarrier carries breathing as phase (PM) and amplitude (AM). */

#include "../../main/bwave_dsp.c"

#include <stdio.h>
#define NEAR_F(a, b) (fabsf((a) - (b)) < 1e-3f)

static int s_fail, s_pass;
#define CHECK(cond, ...) do { \
    if (cond) { s_pass++; } \
    else { s_fail++; printf("FAIL %s:%d: ", __FILE__, __LINE__); \
           printf(__VA_ARGS__); printf("\n"); } \
} while (0)

static int64_t s_now_us;
int64_t esp_timer_get_time(void) { return s_now_us; }
uint8_t bwave_csi_get_node_id(void) { return 7; }

static bwave_resonance_pkt_t s_last_res_pkt;
static int s_res_pkts;
int bwave_stream_send(const uint8_t *data, size_t len)
{
    uint32_t magic;
    memcpy(&magic, data, 4);
    if (magic == BWAVE_RESONANCE_MAGIC && len == sizeof(s_last_res_pkt)) {
        memcpy(&s_last_res_pkt, data, len);
        s_res_pkts++;
    }
    return 0;
}

#define N_SC 32
#define ACTIVE_SC 5

static void run(float seconds, float f_br, float m, float beta)
{
    static uint32_t rng = 99;
    int frames = (int)(seconds * DSP_FS);
    static int t0;
    for (int i = 0; i < frames; i++, t0++) {
        dsp_ring_slot_t slot;
        memset(&slot, 0, sizeof(slot));
        float t = (float)t0 / DSP_FS;
        for (int sc = 0; sc < N_SC; sc++) {
            rng = rng * 1664525u + 1013904223u;
            float jitter = 0.02f * (((float)(rng >> 8) / 16777216.0f) - 0.5f);
            float amp = 40.0f, ph = 0.3f * sc + jitter;
            if (sc == ACTIVE_SC) {
                float s = sinf(2.0f * (float)M_PI * f_br * t);
                amp *= 1.0f + m * s;
                ph += beta * s;
            }
            slot.iq_data[2 * sc]     = (uint8_t)(int8_t)lrintf(amp * cosf(ph));
            slot.iq_data[2 * sc + 1] = (uint8_t)(int8_t)lrintf(amp * sinf(ph));
        }
        slot.iq_len = 2 * N_SC;
        slot.rssi = -50;
        s_now_us += 50000;
        process_frame(&slot);
    }
}

int main(void)
{
    bwave_dsp_config_t cfg = {
        .tier = 2, .vital_interval_ms = 1000, .top_k_count = 8,
    };
    /* init would start a task on target; the stub just returns */
    bwave_dsp_init(&cfg);

    bwave_dsp_filter_t f;
    bwave_dsp_get_filter(&f);
    CHECK(NEAR_F(f.br_f0, sqrtf(0.05f)) && f.br_stages == 1 && f.hr_stages == 1,
          "default filter br f0=%.4f stages=%u/%u", f.br_f0, f.br_stages, f.hr_stages);

    const float f_br = 0.25f, m = 0.10f, beta = 0.40f;
    run(20.0f, f_br, m, beta);

    bwave_dsp_resonance_t r;
    CHECK(bwave_dsp_get_resonance(&r), "resonance measured");
    CHECK(s_res_pkts > 0 && s_last_res_pkt.node_id == 7, "resonance packets sent (%d)", s_res_pkts);
    CHECK(r.br.valid && fabsf(r.br.f0_hz - f_br) < 0.02f, "PM br f0=%.3f valid=%d", r.br.f0_hz, r.br.valid);
    CHECK(r.am_br.valid && fabsf(r.am_br.f0_hz - f_br) < 0.02f, "AM br f0=%.3f valid=%d", r.am_br.f0_hz, r.am_br.valid);
    CHECK(r.am_pm_agree, "AM and PM should agree");
    CHECK(fabsf(r.am_depth - m) < 0.25f * m, "am_depth=%.4f want %.2f", r.am_depth, m);
    CHECK(fabsf(r.pm_index - beta) < 0.25f * beta, "pm_index=%.4f want %.2f", r.pm_index, beta);
    CHECK(r.br_cycles >= 2 && fabsf(r.br_decrement) < 0.1f,
          "steady breathing decrement=%.4f cycles=%u", r.br_decrement, r.br_cycles);
    printf("pipeline: br f0=%.4f q=%.2f%s  am f0=%.4f  am_depth=%.4f (m=%.2f)  pm_index=%.4f (beta=%.2f)  delta=%.4f\n",
           r.br.f0_hz, r.br.q, r.br.res_limited ? " (res-limited)" : "",
           r.am_br.f0_hz, r.am_depth, m, r.pm_index, beta, r.br_decrement);
    CHECK(s_last_res_pkt.flags & BWAVE_RES_F_AGREE, "agree flag in packet 0x%02x", s_last_res_pkt.flags);

    float br, hr;
    bwave_dsp_get_bpm(&br, &hr);
    CHECK(fabsf(br - f_br * 60.0f) < 2.0f, "breathing bpm=%.2f", br);

    /* retune from another task: applied on the next frame */
    bwave_dsp_set_filter(3.0f, -1.0f, 2, 0);
    bwave_dsp_get_filter(&f);
    CHECK(f.br_q == 3.0f && f.br_stages == 2 && f.hr_stages == 1, "requested filter visible");
    run(20.0f, f_br, m, beta);
    CHECK(s_bq_br.n == 2 && s_bq_am.n == 2 && s_person_bq_br[0].n == 2, "cascade stages applied");
    bwave_dsp_get_bpm(&br, &hr);
    CHECK(fabsf(br - f_br * 60.0f) < 2.0f, "bpm after retune=%.2f", br);
    bwave_dsp_get_resonance(&r);
    CHECK(fabsf(r.pm_index - beta) < 0.25f * beta, "pm_index after retune=%.4f", r.pm_index);

    bwave_dsp_set_filter(0.0f, 0.0f, 9, 1);
    bwave_dsp_get_filter(&f);
    CHECK(NEAR_F(f.br_q, sqrtf(0.05f) / 0.4f) && f.br_stages == BWAVE_RES_MAX_STAGES,
          "q=0 restores default, stages clamp (q=%.3f n=%u)", f.br_q, f.br_stages);

    printf("%d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
