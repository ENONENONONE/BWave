#ifndef BWAVE_LCD_H
#define BWAVE_LCD_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    char     name[32];
    uint8_t  position;
    uint8_t  element;
    uint8_t  ray;
    uint8_t  inverse;
    uint8_t  dlog;
    uint8_t  layers_measured;
    char     hash[65];
    char     genome[65];
    bool     active;
} bwave_declared_t;

esp_err_t bwave_lcd_init(void);
void bwave_lcd_show_boot_screen(void);
void bwave_lcd_show_declared_screen(void);
void bwave_lcd_set_declared(const bwave_declared_t *d);
const bwave_declared_t *bwave_lcd_get_declared(void);
void bwave_lcd_update(float hr, float br,
                      float delta, float theta, float alpha,
                      int rssi, int channel, int node_id,
                      float motion, int csi_rate);
void bwave_lcd_update_body(const int8_t *iq, int iq_len);
void bwave_lcd_force_redraw(void);

#endif
