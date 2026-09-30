#pragma once
static inline BaseType_t xTaskCreate(void (*fn)(void *), const char *n, int s,
                                     void *a, int p, void *h)
{ (void)fn; (void)n; (void)s; (void)a; (void)p; (void)h; return pdPASS; }
static inline void vTaskDelay(int t) { (void)t; }
