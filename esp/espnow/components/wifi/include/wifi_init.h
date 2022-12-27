#ifndef WIFI_INIT_H
#define WIFI_INIT_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

void wifi_init(void);

esp_err_t espnow_init(void);

xQueueHandle * get_espnow_queue_handle(void);

#endif // WIFI_INIT_H
