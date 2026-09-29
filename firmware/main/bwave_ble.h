#ifndef BWAVE_BLE_H
#define BWAVE_BLE_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#define BLE_MAX_DEVICES  32

typedef struct {
    uint8_t  mac[6];
    int8_t   rssi;
    char     name[29];
    uint32_t last_seen_ms;
    uint8_t  is_identity;
    uint8_t  position;
    uint8_t  mirrored;
    uint8_t  raw_adv[31];
    uint8_t  raw_adv_len;
} bwave_ble_device_t;

esp_err_t bwave_ble_init(void);
void bwave_ble_scan_start(void);
int bwave_ble_get_device_count(void);
const bwave_ble_device_t *bwave_ble_get_devices(void);
void bwave_ble_get_macs_json(char *buf, size_t buflen);

#endif
