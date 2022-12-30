#include <stdint.h>

#include "espnow_task.h"
#include "espnow_receive_callback.h"
#include "wifi_init.h"
#include "esp_now.h"
#include "esp_log.h"
#include "queues_and_semaphores.h"

#include "freertos/timers.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static uint8_t peer_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#if (CONFIG_TRANSMIT_DEVICE1)

SemaphoreHandle_t timer_semaphore = NULL;

static void periodic_timer_callback(TimerHandle_t xTimer)
{
    xSemaphoreGive(timer_semaphore);
}
#endif // (CONFIG_TRANSMIT_DEVICE1)

#if (CONFIG_RECEIVER_DEVICE)

static const char *GPIO_TAG = "GPIO";

#define GPIO_INPUT_IO_0     (0U)
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0))

SemaphoreHandle_t xGpioSemaphore = NULL;
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    xSemaphoreGiveFromISR(xGpioSemaphore, NULL);
}

void espnow_task(void *pvParameter)
{
    xGpioSemaphore = xSemaphoreCreateBinary();

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL; 
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, NULL);

    for (;;)
    {
        if((xGpioSemaphore != NULL) && (xSemaphoreTake(xGpioSemaphore, 0) == pdTRUE))
        {
            // Write direction register for the MPS MA730GQ device to increase angle when turning counter clockwise
            const uint8_t calibrate_cmd[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
            ESP_LOGI(GPIO_TAG, "Sending calbrate command.\n");
            esp_now_send(peer_mac, calibrate_cmd, 5U);
        }
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}

#elif (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)

void espnow_task(void *pvParameter)
{
#if (CONFIG_TRANSMIT_DEVICE1)
    timer_semaphore = xSemaphoreCreateBinary();

    TimerHandle_t periodic_timer_100Hz;

    periodic_timer_100Hz = xTimerCreate("periodic_timer_100Hz", pdMS_TO_TICKS(10), pdTRUE, NULL, periodic_timer_callback);

    xTimerStart(periodic_timer_100Hz, portMAX_DELAY);
#endif // (CONFIG_TRANSMIT_DEVICE1)

    for(;;)
    {
#if (CONFIG_TRANSMIT_DEVICE1)    
        if((timer_semaphore != NULL) && (xSemaphoreTake(timer_semaphore, 0) == pdTRUE))
        {
            angle_value_to_raw_U angle_to_send = {.value = 0};

            if((mps_comms_queue != NULL) && (xQueueReceive(mps_comms_queue, &angle_to_send.value, 0) == pdTRUE))
            {
                esp_now_send(peer_mac, angle_to_send.raw, 2U);
            }
        }
#endif // (CONFIG_TRANSMIT_DEVICE1)

        received_data_S received_data;

        while((espnow_receive_queue != NULL) && (xQueueReceive(espnow_receive_queue, &received_data, 0) == pdTRUE)) // non-blocking
        {
            switch (received_data.received_data_category)
            {
                case RECEIVED_DATA_CALIBRATE_COMMAND:
                {
                    xSemaphoreGive(mps_calibration_semaphore);
                    break;
                }
                
                case RECEIVED_DATA_ANGLE:
                default:
                {
#if (CONFIG_TRANSMIT_DEVICE2)
                    // Wait for 1ms after the transmit device 1 has sent the message out and then send from transmit device 2
                    vTaskDelay(1 / portTICK_PERIOD_MS);
                    angle_value_to_raw_U angle_to_send = {.value = 0};

                    if((mps_comms_queue != NULL) && (xQueueReceive(mps_comms_queue, &angle_to_send.value, 0) == pdTRUE))
                    {
                        esp_now_send(peer_mac, angle_to_send.raw, 2U);
                    }
#endif // (CONFIG_TRANSMIT_DEVICE2)
                    // received angle process
                    break;
                };
            }
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

#endif // (CONFIG_RECEIVER_DEVICE)
