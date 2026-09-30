#ifndef BWAVE_CONFIG_H
#define BWAVE_CONFIG_H

#include <stdint.h>

#define BWAVE_SSID_MAX  33
#define BWAVE_PASS_MAX  65
#define BWAVE_IP_MAX    16

typedef struct {
    char     wifi_ssid[BWAVE_SSID_MAX];
    char     wifi_password[BWAVE_PASS_MAX];
    char     target_ip[BWAVE_IP_MAX];
    uint16_t target_port;
    uint8_t  node_id;
    uint8_t  wifi_channel;
    uint8_t  edge_tier;
    uint16_t vital_interval_ms;
    float    presence_thresh;
    uint8_t  top_k_count;
    uint8_t  sd_logging;
    uint8_t  lcd_enabled;
    float    br_q;        /* vital filter per-stage Q, 0 = band default */
    float    hr_q;
    uint8_t  br_stages;   /* identical stages in cascade, 1..4 */
    uint8_t  hr_stages;
} bwave_config_t;

void bwave_config_load(bwave_config_t *cfg);

#endif
