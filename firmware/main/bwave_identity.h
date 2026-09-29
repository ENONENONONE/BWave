#ifndef BWAVE_IDENTITY_H
#define BWAVE_IDENTITY_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define BWAVE_IDENTITY_MAGIC  0xC511003E

typedef struct {
    uint8_t position;      /* 0 = unassigned */
    uint8_t element;       /* 3^position mod 257 */
    uint8_t fold_element;  /* element^-1 mod 257 */
    uint8_t ray;           /* position mod 12 */
    uint8_t fold_dlog;     /* 256 - position */
} bwave_self_id_t;

/* Valid after bwave_identity_init(); all zero when unassigned. */
const bwave_self_id_t *bwave_identity_self(void);

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t  node_id;
    uint8_t  position;
    uint8_t  element;
    uint8_t  fold_element;
    uint8_t  ray;
    uint8_t  fold_dlog;
    uint8_t  flags;
    uint8_t  reserved;
    uint16_t heartrate;
    uint16_t breathing_rate;
    float    presence;
    float    motion;
    uint8_t  hash[32];
    uint32_t timestamp_ms;
    uint16_t n_subcarriers;
} bwave_identity_pkt_t;

_Static_assert(sizeof(bwave_identity_pkt_t) == 62, "identity packet = 62 bytes");

void bwave_identity_init(void);
void bwave_identity_send(void);
void bwave_identity_get_json(char *buf, size_t buflen);
const uint8_t *bwave_identity_get_hash(void);

void bwave_identity_save_to_sd(uint8_t position, uint8_t element,
                               uint8_t fold, uint8_t ray,
                               const uint8_t *hash, const char *genome);
void bwave_identity_list_sd(char *buf, size_t buflen);

#endif
