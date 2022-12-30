#include <stdint.h>

#include "queues_and_semaphores.h"
#include "mps_task.h"
#include "espnow_task.h"
#include "mps_comms.h"

#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define GPIO_MOSI 13
#define GPIO_MISO 12
#define GPIO_SCLK 14
#define GPIO_CS 15

#define MPS_QUEUE_SIZE (1)

#if (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)

QueueHandle_t mps_comms_queue = NULL;

SemaphoreHandle_t mps_calibration_semaphore = NULL;

void mps_comms(void *pvParameter)
{
    mps_comms_queue = xQueueCreate(MPS_QUEUE_SIZE, sizeof(uint16_t));

    mps_calibration_semaphore = xSemaphoreCreateBinary();

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

    spi_bus_initialize(HSPI_HOST, &buscfg, 0);

    spi_bus_add_device(HSPI_HOST, &devcfg, &handle);

    for(;;)
    {
        const uint16_t angle_little_endian = mps_get_angle(&handle);

        if (mps_comms_queue != NULL)
        {
            xQueueOverwrite(mps_comms_queue, &angle_little_endian);
        }
        
        if((mps_calibration_semaphore != NULL) && (xSemaphoreTake(mps_calibration_semaphore, 0) == pdTRUE))
        {
            mps_set_zero_position_to_current_position(&handle);
            
            mps_set_ccw_as_incrementing(&handle);
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
#endif // (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)
