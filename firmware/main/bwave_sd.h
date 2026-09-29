#ifndef BWAVE_SD_H
#define BWAVE_SD_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

esp_err_t bwave_sd_init(void);
void bwave_sd_log_vitals(uint8_t node_id, int8_t rssi, float hr, float br,
                         float delta, float theta, float alpha,
                         float motion, const int8_t *iq, int iq_len);
void bwave_sd_flush(void);
bool bwave_sd_is_mounted(void);

#endif
