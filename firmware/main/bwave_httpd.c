#include "bwave_httpd.h"
#include "bwave_sd.h"
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <dirent.h>
#include "esp_log.h"
#include "esp_http_server.h"

static const char *TAG = "bwave_httpd";
static httpd_handle_t s_server = NULL;

#define WWW_ROOT "/sdcard/www"
#define CHUNK_SZ 4096

static const struct {
    const char *ext;
    const char *mime;
} s_mime[] = {
    { ".html", "text/html" },
    { ".htm",  "text/html" },
    { ".css",  "text/css" },
    { ".js",   "application/javascript" },
    { ".json", "application/json" },
    { ".png",  "image/png" },
    { ".jpg",  "image/jpeg" },
    { ".jpeg", "image/jpeg" },
    { ".gif",  "image/gif" },
    { ".svg",  "image/svg+xml" },
    { ".ico",  "image/x-icon" },
    { ".txt",  "text/plain" },
    { ".csv",  "text/csv" },
    { ".pdf",  "application/pdf" },
};

static const char *get_mime(const char *path)
{
    const char *dot = strrchr(path, '.');
    if (!dot) return "application/octet-stream";
    for (int i = 0; i < sizeof(s_mime) / sizeof(s_mime[0]); i++) {
        if (strcasecmp(dot, s_mime[i].ext) == 0)
            return s_mime[i].mime;
    }
    return "application/octet-stream";
}

static esp_err_t file_handler(httpd_req_t *req)
{
    char path[256];
    const char *uri = req->uri;

    /* strip query string */
    const char *q = strchr(uri, '?');
    int uri_len = q ? (int)(q - uri) : (int)strlen(uri);

    /* build path */
    if (uri_len == 1 && uri[0] == '/') {
        snprintf(path, sizeof(path), "%s/index.html", WWW_ROOT);
    } else {
        snprintf(path, sizeof(path), "%s%.*s", WWW_ROOT, uri_len, uri);
    }

    /* block path traversal */
    if (strstr(path, "..")) {
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Forbidden");
        return ESP_OK;
    }

    /* check for .gz companion */
    char gz_path[260];
    snprintf(gz_path, sizeof(gz_path), "%s.gz", path);
    struct stat st;
    bool use_gz = (stat(gz_path, &st) == 0);

    const char *serve_path = use_gz ? gz_path : path;
    if (!use_gz && stat(path, &st) != 0) {
        /* root with no index.html → directory listing */
        if (uri_len == 1 && uri[0] == '/') {
            httpd_resp_set_type(req, "text/html");
            httpd_resp_sendstr_chunk(req,
                "<html><head><title>BWave</title>"
                "<style>body{font:14px monospace;margin:2em;background:#1a1a2e;"
                "color:#e0e0e0}a{color:#d4a843}</style></head>"
                "<body><h2>BWave</h2><ul>");
            DIR *d = opendir(WWW_ROOT);
            if (d) {
                struct dirent *ent;
                while ((ent = readdir(d)) != NULL) {
                    if (ent->d_name[0] == '.') continue;
                    char line[600];
                    snprintf(line, sizeof(line),
                        "<li><a href=\"/%.255s\">%.255s</a></li>",
                        ent->d_name, ent->d_name);
                    httpd_resp_sendstr_chunk(req, line);
                }
                closedir(d);
            }
            httpd_resp_sendstr_chunk(req, "</ul></body></html>");
            httpd_resp_sendstr_chunk(req, NULL);
            return ESP_OK;
        }
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
        return ESP_OK;
    }

    FILE *f = fopen(serve_path, "r");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Read error");
        return ESP_OK;
    }

    httpd_resp_set_type(req, get_mime(path));
    if (use_gz)
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char *chunk = malloc(CHUNK_SZ);
    if (!chunk) {
        fclose(f);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_OK;
    }

    size_t n;
    while ((n = fread(chunk, 1, CHUNK_SZ, f)) > 0) {
        if (httpd_resp_send_chunk(req, chunk, n) != ESP_OK) {
            fclose(f);
            free(chunk);
            httpd_resp_sendstr_chunk(req, NULL);
            return ESP_FAIL;
        }
    }
    fclose(f);
    free(chunk);

    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

esp_err_t bwave_httpd_start(void)
{
    if (!bwave_sd_is_mounted()) {
        ESP_LOGW(TAG, "SD not mounted — HTTP server disabled");
        return ESP_ERR_INVALID_STATE;
    }

    /* ensure www dir exists */
    struct stat st;
    if (stat(WWW_ROOT, &st) != 0) {
        mkdir(WWW_ROOT, 0755);
        ESP_LOGI(TAG, "Created %s", WWW_ROOT);
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.max_uri_handlers = 4;
    config.stack_size = 8192;

    esp_err_t ret = httpd_start(&s_server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    httpd_uri_t file_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = file_handler,
    };
    httpd_register_uri_handler(s_server, &file_uri);

    ESP_LOGI(TAG, "HTTP file server started — serving %s on :80", WWW_ROOT);
    return ESP_OK;
}

void bwave_httpd_stop(void)
{
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
    }
}
