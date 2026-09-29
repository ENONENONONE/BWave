#include "bwave_sd.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"

static const char *TAG = "bwave_sd";

/* Waveshare ESP32-C6 1.47" Display SD card pins */
#define SD_MOSI  GPIO_NUM_6
#define SD_MISO  GPIO_NUM_5
#define SD_CLK   GPIO_NUM_7
#define SD_CS    GPIO_NUM_4

#define MOUNT_POINT "/sdcard"

static FILE *s_log_file = NULL;
static bool s_mounted = false;
static uint32_t s_log_count = 0;

esp_err_t bwave_sd_init(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI,
        .miso_io_num = SD_MISO,
        .sclk_io_num = SD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SPI bus init failed (may be shared): %s", esp_err_to_name(ret));
    }

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = SD_CS;
    slot_cfg.host_id = host.slot;

    sdmmc_card_t *card;
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_cfg, &mount_cfg, &card);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SD mount failed: %s — logging disabled", esp_err_to_name(ret));
        return ret;
    }

    s_mounted = true;
    sdmmc_card_print_info(stdout, card);

    char path[64];
    uint32_t boot_ms = (uint32_t)(esp_timer_get_time() / 1000);
    snprintf(path, sizeof(path), MOUNT_POINT "/bwave_%lu.jsonl", (unsigned long)boot_ms);

    s_log_file = fopen(path, "w");
    if (!s_log_file) {
        ESP_LOGE(TAG, "Failed to open %s", path);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Logging to %s", path);
    return ESP_OK;
}

void bwave_sd_log_vitals(uint8_t node_id, int8_t rssi, float hr, float br,
                         float delta, float theta, float alpha,
                         float motion, const int8_t *iq, int iq_len)
{
    if (!s_log_file) return;

    float t = (float)esp_timer_get_time() / 1000000.0f;

    fprintf(s_log_file,
        "{\"t\":%.3f,\"node_id\":%u,\"rssi\":%d,"
        "\"hr\":%.2f,\"br\":%.2f,"
        "\"delta\":%.6f,\"theta\":%.6f,\"alpha\":%.6f,"
        "\"motion\":%.6f",
        t, (unsigned)node_id, (int)rssi,
        hr, br, delta, theta, alpha, motion);

    if (iq && iq_len > 0) {
        fprintf(s_log_file, ",\"iq\":[");
        int n_pairs = iq_len / 2;
        if (n_pairs > 64) n_pairs = 64;
        for (int i = 0; i < n_pairs; i++) {
            if (i > 0) fputc(',', s_log_file);
            fprintf(s_log_file, "[%d,%d]", (int)iq[i*2], (int)iq[i*2+1]);
        }
        fputc(']', s_log_file);
    }

    fprintf(s_log_file, "}\n");

    s_log_count++;
    if ((s_log_count % 100) == 0) {
        fflush(s_log_file);
    }
}

void bwave_sd_flush(void)
{
    if (s_log_file) fflush(s_log_file);
}

bool bwave_sd_is_mounted(void)
{
    return s_mounted;
}
