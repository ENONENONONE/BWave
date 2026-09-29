#include "bwave_stream.h"
#include <string.h>
#include <errno.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"

static const char *TAG = "bwave_stream";
static int s_sock = -1;
static struct sockaddr_in s_dest;

static int64_t s_backoff_until_us = 0;
#define ENOMEM_COOLDOWN_MS 100

int bwave_stream_init(const char *ip, uint16_t port)
{
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "socket failed: errno %d", errno);
        return -1;
    }

    memset(&s_dest, 0, sizeof(s_dest));
    s_dest.sin_family = AF_INET;
    s_dest.sin_port = htons(port);

    if (inet_pton(AF_INET, ip, &s_dest.sin_addr) <= 0) {
        ESP_LOGE(TAG, "Invalid IP: %s", ip);
        close(s_sock);
        s_sock = -1;
        return -1;
    }

    ESP_LOGI(TAG, "UDP -> %s:%d", ip, port);
    return 0;
}

int bwave_stream_send(const uint8_t *data, size_t len)
{
    if (s_sock < 0) return -1;

    if (s_backoff_until_us > 0) {
        if (esp_timer_get_time() < s_backoff_until_us)
            return -1;
        s_backoff_until_us = 0;
    }

    int sent = sendto(s_sock, data, len, 0,
                      (struct sockaddr *)&s_dest, sizeof(s_dest));
    if (sent < 0) {
        if (errno == ENOMEM)
            s_backoff_until_us = esp_timer_get_time() + (int64_t)ENOMEM_COOLDOWN_MS * 1000;
        return -1;
    }
    return sent;
}

void bwave_stream_deinit(void)
{
    if (s_sock >= 0) {
        close(s_sock);
        s_sock = -1;
    }
}
