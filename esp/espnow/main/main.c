/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example shows how to use ESPNOW.
   Prepare two device, one for sending ESPNOW data and another for receiving
   ESPNOW data.
*/
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_timer.h"

#include "wifi_init.h"
#include "espnow_task.h"

#define GPIO_OUTPUT_IO_18    (18)
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_18)

#define TIMER_DIVIDER         16  //  Hardware timer clock divider

#define RECEIVER_DEVICE (1U)
// #define TRANSMIT_DEVICE1 (1U)
// #define TRANSMIT_DEVICE2 (1U)

#define GPIO_MOSI 13
#define GPIO_MISO 12
#define GPIO_SCLK 14
#define GPIO_CS 15

#if (RECEIVER_DEVICE)
#define GPIO_INPUT_IO_0     (0U)
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0))
#endif // (RECEIVER_DEVICE)

// Local types
union angle_value_to_raw_U
{
    uint16_t value;
    uint8_t raw[2];
};

static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

#if (TRANSMIT_DEVICE1 || TRANSMIT_DEVICE2)
union angle_value_to_raw_U angle_to_send = { .value = 0.0f };
uint8_t calibrate_cmd_set = 0;
#else
union angle_value_to_raw_U angle_received = { .value = 0.0f };
#endif // (TRANSMIT_DEVICE1 || TRANSMIT_DEVICE2)


#if (RECEIVER_DEVICE)
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    // Write direction register for the MPS MA730GQ device to increase angle when turning counter clockwise
    const uint8_t calibrate_cmd[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_send(peer_mac, calibrate_cmd, 5U);
}
#endif // (RECEIVER_DEVICE)

#if (TRANSMIT_DEVICE1)
static void periodic_timer_callback(void* arg)
{
    esp_now_send(peer_mac, angle_to_send.raw, 2U);
}
#endif // (TRANSMIT_DEVICE1)

#if (TRANSMIT_DEVICE1 || TRANSMIT_DEVICE2)
static void mps_comms(void *pvParameter)
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

    for(;;)
    {
        t.tx_data[0] = 0x0;
        t.tx_data[1] = 0x0;
        spi_device_transmit(handle, &t);
        uint16_t angle_little_endian = (t.rx_data[0] << 8U) | t.rx_data[1];
        angle_to_send.value = angle_little_endian;

        if(calibrate_cmd_set)
        {
            calibrate_cmd_set = 0U;

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

            if(t.rx_data[0] == 0x80U)
            {
                ESP_LOGI(TAG, "CCW direction set");
            }
        }
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}
#endif // (TRANSMIT_DEVICE1 || TRANSMIT_DEVICE2)



void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    gpio_config_t io_conf = {0};
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

#if (RECEIVER_DEVICE)
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL; 
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, NULL);
#endif // (RECEIVER_DEVICE)

    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = 0,
    };

    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    wifi_init();
    espnow_init();

#if (TRANSMIT_DEVICE1)
    esp_timer_create_args_t periodic_timer_args = {0};
    periodic_timer_args.callback = (esp_timer_cb_t)periodic_timer_callback;
    periodic_timer_args.arg = NULL;
    periodic_timer_args.dispatch_method = ESP_TIMER_TASK;
    periodic_timer_args.name = "periodic_timer";

    esp_timer_handle_t periodic_timer;

    esp_timer_create(&periodic_timer_args, &periodic_timer);

    esp_timer_start_periodic(periodic_timer, 10000);
#endif // (TRANSMIT_DEVICE1)

    xTaskCreate(espnow_task, "espnow_task", 4096, NULL, 5, NULL);

#if (TRANSMIT_DEVICE1 || TRANSMIT_DEVICE2)
    xTaskCreate(mps_comms, "mps_comms", 4096, NULL, 4, NULL);
#endif // (TRANSMIT_DEVICE1 || TRANSMIT_DEVICE2)
}
