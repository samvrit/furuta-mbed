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
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "espnow_example.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_timer.h"

#define GPIO_OUTPUT_IO_18    (18)
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_18)

#define TIMER_DIVIDER         16  //  Hardware timer clock divider

// #define RECEIVER_DEVICE (1U)
// #define TRANSMIT_DEVICE1 (1U)
#define TRANSMIT_DEVICE2 (1U)

#define GPIO_MOSI 13
#define GPIO_MISO 12
#define GPIO_SCLK 14
#define GPIO_CS 15

#define PI (3.1415f)
#define ANGLE_SCALING_FACTOR (2.0f * PI / 65535.0f)

// Local types
union uint32_to_float_T
{
    float value;
    uint8_t raw[4];
};

static const char *TAG = "espnow_example";

static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

#if (TRANSMIT_DEVICE1 || TRANSMIT_DEVICE2)
union uint32_to_float_T angle_to_send = { .value = 0.0f };
#else
union uint32_to_float_T angle_received = { .value = 0.0f };
#endif // (TRANSMIT_DEVICE1 || TRANSMIT_DEVICE2)

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(ESPNOW_WIFI_MODE);
    esp_wifi_start();
    esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR);
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}

static void example_espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
#if (TRANSMIT_DEVICE2)  // For transmit device 2, wait for 1ms after transmit device 1 has broadcast its position so as to not interfere

    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    timer_start(TIMER_GROUP_0, TIMER_0);

    // Wait for 1ms
    // HW timer peripheral uses 80MHz clock as the base, and the divider is set to 16
    // Hence, 1ms = 0.001 * (80000000/16) = 5000 counts
    uint64_t timer_val = 0UL;
    while(timer_val < 5000UL)
    {
        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timer_val);
    }
    timer_pause(TIMER_GROUP_0, TIMER_0);

    esp_now_send(peer_mac, angle_to_send.raw, 4U);

#elif (RECEIVER_DEVICE)

    for(uint8_t i = 0; i < 4; i++)
    {
        angle_received.raw[i] = data[i];
    }

    ESP_LOGI(TAG, "Angle received from %02X %02X %02X %02X %02X %02X: %f\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], angle_received.value);

#endif // RECEIVER_DEVICE
}

#if (TRANSMIT_DEVICE1)
static void periodic_timer_callback(void* arg)
{
    esp_now_send(peer_mac, angle_to_send.raw, 4U);
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
        spi_device_transmit(handle, &t);
        uint16_t angle_little_endian = (t.rx_data[0] << 8U) | t.rx_data[1];
        const float angle = angle_little_endian * ANGLE_SCALING_FACTOR;
        vTaskDelay(1 / portTICK_RATE_MS);
        angle_to_send.value = angle;
    }
}
#endif // (TRANSMIT_DEVICE1 || TRANSMIT_DEVICE2)

static esp_err_t example_espnow_init(void)
{
    /* Initialize ESPNOW and register sending and receiving callback function. */
    esp_now_init();
    esp_now_register_send_cb(example_espnow_send_cb);
    esp_now_register_recv_cb(example_espnow_recv_cb);

    /* Set primary master key. */
    esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK);

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t peer = {0};
    peer.channel = CONFIG_ESPNOW_CHANNEL;
    peer.ifidx = ESPNOW_WIFI_IF;
    peer.encrypt = false;

    for (uint8_t i = 0; i < ESP_NOW_ETH_ALEN; i++)
    {
        peer.peer_addr[i] = peer_mac[i];
    }

    esp_now_add_peer(&peer);

    return ESP_OK;
}

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

    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = 0,
    };

    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    example_wifi_init();
    example_espnow_init();

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

#if (TRANSMIT_DEVICE1 || TRANSMIT_DEVICE2)
    xTaskCreate(mps_comms, "mps_comms", 4096, NULL, 4, NULL);
#endif // (TRANSMIT_DEVICE1 || TRANSMIT_DEVICE2)
}
