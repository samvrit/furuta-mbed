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
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_timer.h"

#define ESPNOW_MAXDELAY 512

#define GPIO_OUTPUT_IO_0    18
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

static const char *TAG = "espnow_example";

static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = { 0x24, 0x0A, 0xC4, 0xEC, 0xA6, 0x28 };

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
    gpio_set_level(GPIO_OUTPUT_IO_0, 1U);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    timer_start(TIMER_GROUP_0, TIMER_0);

    uint64_t timer_val = 0UL;
    while(timer_val < 10UL)
    {
        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timer_val);
    }
    timer_pause(TIMER_GROUP_0, TIMER_0);
    
    gpio_set_level(GPIO_OUTPUT_IO_0, 0U);
}

static void periodic_timer_callback(void* arg)
{
    gpio_set_level(GPIO_OUTPUT_IO_0, 1U);
    const uint8_t data_to_send[4] = {0x12, 0x23, 0x34, 0x56};
    esp_now_send(peer_mac, data_to_send, 4U);
    gpio_set_level(GPIO_OUTPUT_IO_0, 0U);
}

static void example_espnow_task(void *pvParameter)
{
    vTaskDelay(1000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    esp_timer_create_args_t periodic_timer_args = {0};
    periodic_timer_args.callback = (esp_timer_cb_t)periodic_timer_callback;
    periodic_timer_args.arg = NULL;
    periodic_timer_args.dispatch_method = ESP_TIMER_TASK;
    periodic_timer_args.name = "periodic_timer";

    esp_timer_handle_t periodic_timer;

    esp_timer_create(&periodic_timer_args, &periodic_timer);

    esp_timer_start_periodic(periodic_timer, 10000);

    for(;;)
    {
    }
}

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

    xTaskCreate(example_espnow_task, "example_espnow_task", 2048, NULL, 14, NULL);
}
