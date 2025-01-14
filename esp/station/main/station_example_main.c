/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/spi_master.h"
#include "esp32/rom/ets_sys.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>

/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define WIFI_SSID                   "ublox"
#define WIFI_PASS                   "furuta123"
#define MAXIMUM_RETRY               5
#define LOCAL_IP_ADDR_DEVICE_1      "10.0.0.5"
#define LOCAL_IP_ADDR_DEVICE_2      "10.0.0.10"
#define GATEWAY_ADDR                "10.0.0.1"
#define MAX_SOCKET_RETRIES          5
#define LOCAL_PORT                  8888
#define SERVER_PORT                 5050

#define GPIO_MOSI 13
#define GPIO_MISO 12
#define GPIO_SCLK 14
#define GPIO_CS 15

#define PI 3.1415F
#define ANGLE_SCALING_FACTOR (-PI/32768U) // reverse sign to comply with model

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *WIFI_TAG = "WIFI";
static const char *UDP_TAG = "UDP";

static int s_retry_num = 0;
static uint8_t device1;

typedef union {
    float value;
    char buffer[sizeof(float)];
} udpPacket_t;

volatile udpPacket_t udpPacket;

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) 
    {
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) 
    {
        if (s_retry_num < MAXIMUM_RETRY) 
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(WIFI_TAG, "retry to connect to the AP");
        } 
        else 
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(WIFI_TAG,"connect to the AP fail");
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) 
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(WIFI_TAG, "got ip:%s", ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta()
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA)); // stop DHCP server for the AP mode
    // assign a static IP to the network interface

    uint8_t mac_address[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(mac_address, 0));

    ESP_LOGW(WIFI_TAG, "MAC: %02X %02X %02X %02X %02X %02X", mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5]);

    tcpip_adapter_ip_info_t info;
    memset(&info, 0, sizeof(info));  

    device1 = ((mac_address[3] == 0x7A) && (mac_address[4] == 0x10) && (mac_address[5] == 0xEC)) ? 1 : 0; // use MAC address to differentiate between the two devices  

    if(device1)
    {
        info.ip.addr = inet_addr(LOCAL_IP_ADDR_DEVICE_1);
    }
    else
    {
        info.ip.addr = inet_addr(LOCAL_IP_ADDR_DEVICE_2);
    }
        
    info.gw.addr = inet_addr(GATEWAY_ADDR);
    IP4_ADDR(&info.netmask, 255, 255, 255, 0);

    ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA, &info)); // hook the TCP adapter to the WiFi driver

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(WIFI_TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) 
    {
        ESP_LOGI(WIFI_TAG, "connected to ap SSID:%s password:%s",WIFI_SSID, WIFI_PASS);
    } 
    else if (bits & WIFI_FAIL_BIT) 
    {
        ESP_LOGI(WIFI_TAG, "Failed to connect to SSID:%s, password:%s", WIFI_SSID, WIFI_PASS);
    } 
    else 
    {
        ESP_LOGE(WIFI_TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}

static void udp_server_task(void *pvParameters)
{
    int socket_retry_count = 0;

    spi_device_handle_t handle;
    esp_err_t spi_ret;

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
        .command_bits=0,
        .address_bits=0,
        .dummy_bits=0,
        .clock_speed_hz=25000000,
        .duty_cycle_pos=128,        //50% duty cycle
        .mode=0,
        .spics_io_num=GPIO_CS,
        .cs_ena_posttrans=3,        //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size=3
    };

    int16_t angle_raw;
    uint16_t angle_cmd = 0x0000;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.length = 16U; // 16 bits
    t.rx_buffer = &angle_raw;
    t.tx_buffer = &angle_cmd;

    //Initialize the SPI bus and add the device we want to send stuff to.
    spi_ret = spi_bus_initialize(HSPI_HOST, &buscfg, 0);
    assert(spi_ret==ESP_OK);
    spi_ret = spi_bus_add_device(HSPI_HOST, &devcfg, &handle);
    assert(spi_ret==ESP_OK);

    while (socket_retry_count < MAX_SOCKET_RETRIES) 
    {
        
        struct sockaddr_in local_addr;
        if(device1)
        {
            local_addr.sin_addr.s_addr = inet_addr(LOCAL_IP_ADDR_DEVICE_1);
        }
        else
        {
            local_addr.sin_addr.s_addr = inet_addr(LOCAL_IP_ADDR_DEVICE_2);
        }
        local_addr.sin_family = AF_INET;
        local_addr.sin_port = htons(LOCAL_PORT);

        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (sock < 0) 
        {
            ESP_LOGE(UDP_TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(UDP_TAG, "Socket created");

        int err = bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr));
        if (err < 0) 
        {
            ESP_LOGE(UDP_TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(UDP_TAG, "Socket bound, port %d", LOCAL_PORT);

        struct sockaddr_in server_addr; 
        server_addr.sin_addr.s_addr = inet_addr(GATEWAY_ADDR);   // 10.0.0.1
        server_addr.sin_port = htons(SERVER_PORT);
        server_addr.sin_family = AF_INET;
        socklen_t socklen = sizeof(server_addr);
        
        while (1) 
        {
            spi_ret = spi_device_transmit(handle, &t);
            angle_raw = ((angle_raw & 0xFF00U) >> 8) | ((angle_raw & 0x00FFU) << 8);    // convert LSB to MSB first
            float angle = (float)(angle_raw * ANGLE_SCALING_FACTOR);
            // printf("Received: %d, %.5f\n", angle_raw, angle);

            udpPacket.value = angle;

            // try sending a packet as long as the transmit buffer is free
            while(sendto(sock, (const void *)udpPacket.buffer, sizeof(udpPacket.buffer), 0, (struct sockaddr *)&server_addr, socklen) < 0)
            {
                if(12U != errno)    // if error is not related to transmit buffer being full, show the error and break
                {
                    ESP_LOGE(UDP_TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
        }

        if (sock != -1) 
        {
            ESP_LOGE(UDP_TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }

        socket_retry_count++;
        
    }
    vTaskDelete(NULL);
}

void app_main()
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(WIFI_TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 6, NULL);
}
