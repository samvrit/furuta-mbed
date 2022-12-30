#include <stdint.h>
#include <string.h>

#include "c2000_comms_task.h"
#include "queues_and_semaphores.h"
#include "espnow_receive_callback.h"

#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define GPIO_MOSI 13
#define GPIO_MISO 12
#define GPIO_SCLK 14
#define GPIO_CS 15

#if (CONFIG_RECEIVER_DEVICE)

static const char *RECEIVER_TAG = "RECEIVER";

void c2000_comms(void *pvParameter)
{
    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg={
        .mode=0,
        .spics_io_num=GPIO_CS,
        .queue_size=1,
        .flags=0,
        .post_setup_cb=NULL,
        .post_trans_cb=NULL
    };

    //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    spi_slave_initialize(HSPI_HOST, &buscfg, &slvcfg, 0);

    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    uint32_t angles_combined = 0U;

    for(;;)
    {
        received_data_S received_data;

        while((espnow_receive_queue != NULL) && (xQueueReceive(espnow_receive_queue, &received_data, 0) == pdTRUE)) // non-blocking
        {
            switch (received_data.received_data_category)
            {
                case RECEIVED_DATA_ANGLE:
                {
                    ESP_LOGI(RECEIVER_TAG, "%02X: %u\n", received_data.mac[5], received_data.angle.value);

                    if(received_data.mac[5] == 0xEC)    // from transmit device 1
                    {
                        angles_combined |= received_data.angle.value;
                    }
                    else //if (received_data.mac[5] == 0x28)  // from transmit device 2
                    {
                        angles_combined |= ((uint32_t)received_data.angle.value << 16U);
                    }
                    break;
                }
                default:
                {
                    // do nothing
                    break;
                }
            }
        }

        uint8_t recvbuf[4];
        memset(recvbuf, 0x0, 4*sizeof(uint8_t));

        t.length=32;
        t.tx_buffer=(const void *)&angles_combined;
        t.rx_buffer=recvbuf;

        // spi_slave_transmit(HSPI_HOST, &t, portMAX_DELAY);

        taskYIELD();
    }
}
#endif // (CONFIG_RECEIVER_DEVICE)
