#include <stdint.h>

#include "espnow_task.h"
#include "espnow_receive_callback.h"
#include "wifi_init.h"
#include "esp_now.h"

#include "freertos/timers.h"
#include "driver/gpio.h"


#define MPS_QUEUE_SIZE (1)

#if (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)
xQueueHandle mps_comms_queue;
SemaphoreHandle_t mps_calibration_semaphore = NULL;
#endif // (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)

static uint8_t peer_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#if (CONFIG_TRANSMIT_DEVICE1)
SemaphoreHandle_t timer_semaphore = NULL;
TimerHandle_t periodic_timer_100Hz;

static void periodic_timer_callback(TimerHandle_t xTimer)
{
    xSemaphoreGive(timer_semaphore);
}
#endif // (CONFIG_TRANSMIT_DEVICE1)

#if (CONFIG_RECEIVER_DEVICE)
#define GPIO_INPUT_IO_0     (0U)
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0))

SemaphoreHandle_t xGpioSemaphore = NULL;
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    xSemaphoreGiveFromISR(xGpioSemaphore, NULL);
}
#endif // (CONFIG_RECEIVER_DEVICE)

void espnow_task(void *pvParameter)
{
    received_data_S received_data;

    xQueueHandle * espnow_queue_handle = get_espnow_queue_handle();

#if (CONFIG_RECEIVER_DEVICE)
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL; 
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, NULL);
#endif // (CONFIG_RECEIVER_DEVICE)

#if (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)
    mps_comms_queue = xQueueCreate(MPS_QUEUE_SIZE, sizeof(uint16_t));
    mps_calibration_semaphore = xSemaphoreCreateBinary();
#endif // (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)

#if (CONFIG_TRANSMIT_DEVICE1)
    timer_semaphore = xSemaphoreCreateBinary();

    periodic_timer_100Hz = xTimerCreate("periodic_timer_100Hz", pdMS_TO_TICKS(10), pdTRUE, NULL, periodic_timer_callback);

    xTimerStart(periodic_timer_100Hz, portMAX_DELAY);
#endif // (CONFIG_TRANSMIT_DEVICE1)

    for(;;)
    {
#if (CONFIG_RECEIVER_DEVICE)
        if(xSemaphoreTake(xGpioSemaphore, 0) == pdTRUE)
        {
            // Write direction register for the MPS MA730GQ device to increase angle when turning counter clockwise
            const uint8_t calibrate_cmd[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
            esp_now_send(peer_mac, calibrate_cmd, 5U);
        }
#endif // (CONFIG_RECEIVER_DEVICE)

#if (CONFIG_TRANSMIT_DEVICE1)    
        if(xSemaphoreTake(timer_semaphore, 0) == pdTRUE)
        {
            angle_value_to_raw_U angle_to_send = {.value = 0};

            if(xQueueReceive(mps_comms_queue, &angle_to_send.value, 0) == pdTRUE)
            {
                esp_now_send(peer_mac, angle_to_send.raw, 2U);
            }
        }
#endif // (CONFIG_TRANSMIT_DEVICE1)

        if(xQueueReceive(*espnow_queue_handle, &received_data, 0) == pdTRUE) // non-blocking
        {
            switch (received_data.received_data_category)
            {
                case RECEIVED_DATA_CALIBRATE_COMMAND:
                {
#if (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)
                    xSemaphoreGive(mps_calibration_semaphore);
#endif // (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)
                    break;
                }
                
                case RECEIVED_DATA_ANGLE:
                default:
                {
#if (CONFIG_TRANSMIT_DEVICE2)
                    // Wait for 1ms after the transmit device 1 has sent the message out and then send from transmit device 2
                    vTaskDelay(1 / portTICK_RATE_MS);
                    angle_value_to_raw_U angle_to_send = {.value = 0};

                    if(xQueueReceive(mps_comms_queue, &angle_to_send.value, 0) == pdTRUE)
                    {
                        esp_now_send(peer_mac, angle_to_send.raw, 2U);
                    }
#elif (CONFIG_RECEIVER_DEVICE)
                    // receiver device 
#endif // (CONFIG_TRANSMIT_DEVICE2)
                    // received angle process
                    break;
                };
            }
        }
    }
}

#if (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)
xQueueHandle * espnow_get_mps_queue_handle(void)
{
    return &mps_comms_queue;
}

SemaphoreHandle_t * espnow_get_mps_calibrate_semaphore_handle(void)
{
    return &mps_calibration_semaphore;
}
#endif // (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)
