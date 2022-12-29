#ifndef QUEUES_AND_SEMAPHORES_H
#define QUEUES_AND_SEMAPHORES_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

extern QueueHandle_t espnow_receive_queue; 

#if (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)
extern QueueHandle_t mps_comms_queue;

extern SemaphoreHandle_t mps_calibration_semaphore;
#endif // (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)

#endif // QUEUES_AND_SEMAPHORES_H
