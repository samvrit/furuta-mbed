#include <stdint.h>

#include "mps_task.h"
#include "espnow_task.h"

#include "freertos/timers.h"
#include "driver/spi_master.h"

#define GPIO_MOSI 13
#define GPIO_MISO 12
#define GPIO_SCLK 14
#define GPIO_CS 15

#if (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)

void mps_comms(void *pvParameter)
{
    spi_device_handle_t handle;

    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };

    //Configuration for the SPI device on the other side of the bus
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=25000000,
        .mode=0,
        .spics_io_num=GPIO_CS,
        .cs_ena_posttrans=3,        //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size=3
    };

    spi_transaction_t t;

    t.length = 16U;
    t.flags = (SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA);

    for(uint8_t i = 0U; i < 4U; i++)
    {
        t.tx_data[i] = 0x0U;
    }

    spi_bus_initialize(HSPI_HOST, &buscfg, 0);

    spi_bus_add_device(HSPI_HOST, &devcfg, &handle);

    xQueueHandle * mps_queue_handle = espnow_get_mps_queue_handle();
    SemaphoreHandle_t * mps_calibrate_semaphore_handle = espnow_get_mps_calibrate_semaphore_handle();

    for(;;)
    {
        t.tx_data[0] = 0x0;
        t.tx_data[1] = 0x0;
        spi_device_transmit(handle, &t);
        uint16_t angle_little_endian = (t.rx_data[0] << 8U) | t.rx_data[1];

        if (mps_queue_handle != NULL)
        {
            xQueueOverwrite(*mps_queue_handle, &angle_little_endian);
        }
        
        if(xSemaphoreTake(*mps_calibrate_semaphore_handle, 0) == pdTRUE)
        {
            const uint32_t angle_to_set = (((uint32_t)angle_little_endian + 32768U) % 65535U);

            const uint8_t angle_to_set_bits_0_7 = (angle_to_set & 0xFFU);
            const uint8_t angle_to_set_bits_8_15 = ((angle_to_set & 0xFF00U) >> 8U);

            t.tx_data[0] = 0x80U;
            t.tx_data[1] = angle_to_set_bits_0_7;

            spi_device_transmit(handle, &t);

            vTaskDelay(20 / portTICK_RATE_MS);

            // acknowledge 
            t.tx_data[0] = 0x0U;
            t.tx_data[1] = 0x0U;
            spi_device_transmit(handle, &t);

            vTaskDelay(1 / portTICK_RATE_MS);

            t.tx_data[0] = 0x81U;
            t.tx_data[1] = angle_to_set_bits_8_15;

            spi_device_transmit(handle, &t);

            vTaskDelay(20 / portTICK_RATE_MS);

            // acknowledge 
            t.tx_data[0] = 0x0U;
            t.tx_data[1] = 0x0U;
            spi_device_transmit(handle, &t);

            vTaskDelay(1 / portTICK_RATE_MS);
            
            // set counter clockwise as the direction in which the angle increases
            t.tx_data[0] = 0x89U;
            t.tx_data[1] = 0x80U;
            spi_device_transmit(handle, &t);

            vTaskDelay(20 / portTICK_RATE_MS);

            // acknowledge 
            t.tx_data[0] = 0x0U;
            t.tx_data[1] = 0x0U;
            spi_device_transmit(handle, &t);
        }
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}
#endif // (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)