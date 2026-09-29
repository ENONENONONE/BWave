#ifndef BWAVE_CSI_H
#define BWAVE_CSI_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#define CSI_MAGIC       0xC5110001
#define CSI_HEADER_SIZE 26

void bwave_csi_set_node_id(uint8_t node_id);
uint8_t bwave_csi_get_node_id(void);
void bwave_csi_init(void);
esp_err_t bwave_csi_inject_ndp(void);

typedef struct {
    int      rssi;
    int      channel;
    uint32_t cb_count;
    uint32_t send_ok;
    uint32_t send_fail;
    int      iq_len;
    int8_t   iq[512];
} bwave_csi_snapshot_t;

void bwave_csi_get_snapshot(bwave_csi_snapshot_t *out);
int  bwave_csi_get_mac_table(uint8_t macs[][6], int8_t *rssi,
                             uint16_t *counts, int max);

#endif
