#ifndef BWAVE_STREAM_H
#define BWAVE_STREAM_H

#include <stdint.h>
#include <stddef.h>

int bwave_stream_init(const char *ip, uint16_t port);
int bwave_stream_send(const uint8_t *data, size_t len);
void bwave_stream_deinit(void);

#endif
