#ifndef BWAVE_CMD_H
#define BWAVE_CMD_H

#include <stddef.h>

void bwave_cmd_init(void);
void bwave_cmd_handle_line(const char *line, char *resp, size_t resp_len);

#endif
