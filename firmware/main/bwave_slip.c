#include "bwave_slip.h"
#include "bwave_cmd.h"
#include "hal/usb_serial_jtag_ll.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "bwave_serial";

static bool s_up = false;
static char s_line[256];
static int  s_len = 0;

static void serial_reader_task(void *arg)
{
    (void)arg;
    uint8_t c;

    ESP_LOGI(TAG, "Serial reader started (LL direct)");

    while (1) {
        int n = usb_serial_jtag_ll_read_rxfifo(&c, 1);
        if (n <= 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        if (c == '\n' || c == '\r') {
            if (s_len > 0) {
                s_line[s_len] = '\0';
                ESP_LOGI(TAG, "CMD: %s", s_line);
                static char resp[2048];
                resp[0] = '\0';
                bwave_cmd_handle_line(s_line, resp, sizeof(resp));
                if (resp[0]) {
                    printf("%s", resp);
                    fflush(stdout);
                }
            }
            s_len = 0;
            continue;
        }

        if (s_len < (int)sizeof(s_line) - 1)
            s_line[s_len++] = (char)c;
    }
}

bool bwave_slip_is_up(void)
{
    return s_up;
}

esp_err_t bwave_slip_init(void)
{
    xTaskCreate(serial_reader_task, "serial_rx", 4096, NULL, 6, NULL);

    s_up = true;
    ESP_LOGI(TAG, "Serial command reader ready (LL direct)");
    return ESP_OK;
}
