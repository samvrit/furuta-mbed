#ifndef ESPNOW_TASK_H
#define ESPNOW_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

void espnow_task(void *pvParameter);

#if (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)
xQueueHandle * espnow_get_mps_queue_handle(void);
SemaphoreHandle_t * espnow_get_mps_calibrate_semaphore_handle(void);
#endif // (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)

#endif // ESPNOW_TASK_H
