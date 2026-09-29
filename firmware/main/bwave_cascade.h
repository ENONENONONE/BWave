#ifndef BWAVE_CASCADE_H
#define BWAVE_CASCADE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define GF_P  257
#define GF_G  3
#define MAX_MIRROR_DEVICES  10
#define MAX_DEVICE_POS      32

/* ── ISA: three flips ──────────────────────────────────────────
 *  F  (0→1): NOP → IDENT     — wake
 *  T  (8→6): SYNC → FOLD     — DARK→ACTIVE crossing
 *  PM (1→0): IDENT → NOP     — sleep
 *  fold is involution: fold(fold(x)) = x — parity preserved
 */
#define FLIP_F   0
#define FLIP_T   1
#define FLIP_PM  2

/* TL operations (opcode → transmission line dual) */
#define TL_HALF    1   /* λ/2  Identity   (op 1 IDENT) */
#define TL_EULER   2   /* √ε   Euler      (op 2 EXIST) */
#define TL_NEGATE  6   /* 257−p Negate    (op 6 FOLD)  */
#define TL_QUARTER 7   /* λ/4  Inverse    (op 7 JUMP)  */
#define TL_DLOG    8   /* dlog Log        (op 8 SYNC)  */

typedef struct {
    float    energy[GF_P];
    float    xray;
    uint8_t  flip_state;       /* 0=sleep 1=wake */
    uint32_t flip_count[3];    /* F, T, PM */
    bool     nano2_active;
    uint16_t nano2_pub;
    uint16_t nano2_inv;
    uint16_t nano2_fold;
    uint16_t nano2_peer;
    char     nano2_name[32];
} bwave_field_state_t;

uint16_t bwave_cascade_dlog(uint16_t pos);
bool     bwave_cascade_is_qr(uint16_t pos);

uint16_t bwave_cascade_inv(uint16_t p);
uint16_t bwave_cascade_neg(uint16_t p);
uint16_t bwave_cascade_ray(uint16_t p);

uint16_t bwave_cascade_norm(uint16_t a, uint16_t b);
uint16_t bwave_cascade_trace(uint16_t a, uint16_t b);

uint16_t bwave_cascade_quality(uint16_t p);
uint16_t bwave_cascade_impedance(uint16_t p);
uint16_t bwave_cascade_reflection(uint16_t p);

void bwave_cascade_handle(const char *json, char *buf, size_t buflen);
void bwave_cascade_store_fold_handle(const char *json, char *buf, size_t buflen);
void bwave_cascade_mirror_handle(const char *json, char *buf, size_t buflen);
void bwave_axiom_seed_handle(const char *json, char *buf, size_t buflen);
int  bwave_cascade_broadcast_sd(char *buf, size_t buflen);
int  bwave_cascade_last_broadcast_count(void);
int  bwave_cascade_get_mirror_keys(uint16_t *keys, int max_keys);
void bwave_cascade_store_device(const char *name,
                                const uint16_t *positions, int npos);
int  bwave_cascade_get_device_count(void);
int  bwave_cascade_get_device_info(int idx, char *name, int name_sz,
                                   uint16_t *positions, int max_pos);
void bwave_cascade_field_analysis(char *buf, size_t buflen);

void bwave_cascade_handle_xray(const char *line);
void bwave_cascade_handle_identity_line(const char *line);
void bwave_cascade_handle_upload_line(const char *line);
void bwave_cascade_handle_field_line(const char *line);
void bwave_cascade_handle_genome_line(const char *line);
void bwave_cascade_handle_doubling_line(const char *line);
void bwave_cascade_field_tick(float decay);
void bwave_cascade_nano2_activate(uint16_t pub, uint16_t inv,
                                  uint16_t fold, uint16_t peer,
                                  const char *name);
const bwave_field_state_t *bwave_cascade_get_field(void);
uint8_t bwave_cascade_tl_op(uint16_t pos);
const char *bwave_cascade_tl_name(uint8_t op);

#endif
