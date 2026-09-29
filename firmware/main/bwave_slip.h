#ifndef BWAVE_SLIP_H
#define BWAVE_SLIP_H

#include "esp_err.h"
#include <stdbool.h>

#define SLIP_ADDR_LOCAL  "10.0.0.1"
#define SLIP_ADDR_PEER   "10.0.0.2"
#define SLIP_CMD_PORT    5006

esp_err_t bwave_slip_init(void);
bool bwave_slip_is_up(void);

#endif
