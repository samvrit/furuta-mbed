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

#define GPIO_CS_IO_CONFIG ((1ULL) << (GPIO_CS))

#define SPI_MS_TO_WAIT (100 / portTICK_RATE_MS)

#if (CONFIG_RECEIVER_DEVICE)

SemaphoreHandle_t spi_cs_interrupt_semaphore = NULL;

static void IRAM_ATTR spi_cs_isr_handler(void* arg)
{
    xSemaphoreGiveFromISR(spi_cs_interrupt_semaphore, NULL);
}

void c2000_comms(void *pvParameter)
{
    spi_cs_interrupt_semaphore = xSemaphoreCreateBinary();

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
        .queue_size=3,
        .flags=0,
        .post_setup_cb=NULL,
        .post_trans_cb=NULL
    };

    //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    io_conf.pin_bit_mask = GPIO_CS_IO_CONFIG; 
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(GPIO_CS, spi_cs_isr_handler, NULL);

    spi_slave_initialize(HSPI_HOST, &buscfg, &slvcfg, 0);

    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.length = 32;

    // blocking SPI slave transmit to ensure that the host is ready
    spi_slave_transmit(HSPI_HOST, &t, portMAX_DELAY);

    uint16_t angle1 = 0U;
    uint16_t angle2 = 0U;
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
                    if(received_data.mac[5] == 0xEC)    // from transmit device 1
                    {
                        angle1 = received_data.angle.value;
                    }
                    else //if (received_data.mac[5] == 0x28)  // from transmit device 2
                    {
                        angle2 = received_data.angle.value;
                    }
                    break;
                }
                default:
                {
                    // do nothing
                    break;
                }
            }

            taskYIELD();
        }

        taskYIELD();

        if((spi_cs_interrupt_semaphore != NULL) && (xSemaphoreTake(spi_cs_interrupt_semaphore, 0) == pdTRUE))
        {
            WORD_ALIGNED_ATTR uint8_t recvbuf[4] = {0U};
            WORD_ALIGNED_ATTR uint8_t sendbuf[4] = {0U};

            if( (angle1 <= 16383U) && (angle2 <= 16383U) )
            {
                angles_combined = ((uint32_t)angle2 << 16U) | angle1;
            }

            memcpy(sendbuf, &angles_combined, 4*sizeof(uint8_t));

            t.length = 32;
            t.tx_buffer=sendbuf;
            t.rx_buffer=recvbuf;

            spi_slave_transmit(HSPI_HOST, &t, SPI_MS_TO_WAIT);
        }

        taskYIELD();
    }
}
#endif // (CONFIG_RECEIVER_DEVICE)
