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
#include "esp_system.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "driver/timer.h"
#include "esp_timer.h"

#include "wifi_init.h"
#include "espnow_task.h"
#include "mps_task.h"

#define TIMER_DIVIDER         16  //  Hardware timer clock divider

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    wifi_init();
    espnow_init();

    xTaskCreate(espnow_task, "espnow_task", 4096, NULL, 5, NULL);

#if (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)
    xTaskCreate(mps_comms, "mps_comms", 4096, NULL, 4, NULL);
#endif // (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)
}
