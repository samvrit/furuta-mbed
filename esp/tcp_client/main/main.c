/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "driver/gpio.h"
#include "driver/spi_slave.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#define WIFI_SSID                   "ublox"
#define WIFI_IP_ADDR                "10.0.0.1"
#define DEVICE_1_IP                 "10.0.0.5"
#define DEVICE_2_IP                 "10.0.0.10"
#define DEVICE_PORT                 8888
#define MAX_STA_CONN                5
#define MAX_SOCKET_RETRIES          5

#define GPIO_MOSI 12
#define GPIO_MISO 13
#define GPIO_SCLK 15
#define GPIO_CS 14

#define UDP_PACKET_DELAY_THRESHOLD  200U    // microseconds
#define X2_DATA_VALID_BIT           1U
#define X3_DATA_VALID_BIT           0U

static const char *WIFI_TAG = "WIFI";
static const char *TCP_TAG = "TCP";

static const uint16_t position_request = 0x10U;
static const uint16_t zero_set_request = 0x80U;

typedef union {
    float value;
    char  buffer[sizeof(float)];
} udpPacket_t;

volatile udpPacket_t x2, x3;

typedef struct {
    float       x2;
    float       x3;
    uint8_t     dataValid;
} sensorData_S;

typedef union {
    sensorData_S sensorData;
    uint32_t bytes[sizeof(sensorData_S)];
} sendBuffer_t;

volatile sendBuffer_t sendBuffer;

uint64_t t0_x2, t0_x3;
uint64_t t1_x2, t1_x3;
uint64_t t_x2, t_x3;

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) 
    {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(WIFI_TAG, "station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);
    } 
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) 
    {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(WIFI_TAG, "station "MACSTR" leave, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap()
{
    esp_netif_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_config_t cfg_netif = ESP_NETIF_DEFAULT_WIFI_AP();
    esp_netif_t *ap_netif = esp_netif_new(&cfg_netif);
    esp_netif_attach_wifi_ap(ap_netif);
    esp_wifi_set_default_wifi_ap_handlers();
    esp_netif_dhcps_stop(ap_netif);

    // assign a static IP to the network interface
    esp_netif_ip_info_t info;
    memset(&info, 0, sizeof(info));
    info.ip.addr = inet_addr(WIFI_IP_ADDR);
    info.gw.addr = inet_addr(WIFI_IP_ADDR);
    IP4_ADDR(&info.netmask, 255, 255, 255, 0);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &info)); // hook the Net-IF to the WiFi AP

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = 
    {
        .ap = 
        {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = "",
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_OPEN
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(WIFI_TAG, "wifi_init_softap finished. SSID:%s", WIFI_SSID);   
}

static __always_inline void process_data(unsigned char *recv_buf, const uint32_t addr_raw)
{
    if(0x0A00000A == addr_raw)
    {
        memcpy((void *)x2.buffer, recv_buf, 4);
        t1_x2 = esp_timer_get_time();
        t_x2 = t1_x2 - t0_x2;
        t0_x2 = t1_x2;
        sendBuffer.sensorData.x2 = x2.value;
        sendBuffer.sensorData.dataValid = (t_x2 <= UDP_PACKET_DELAY_THRESHOLD) ? (sendBuffer.sensorData.dataValid | (1 << X2_DATA_VALID_BIT)) : (sendBuffer.sensorData.dataValid & ~(1 << X2_DATA_VALID_BIT));
        ESP_LOGI(TCP_TAG, "%lld", t_x2);
    }
    else if(0x0500000A == addr_raw)
    {
        memcpy((void *)x3.buffer, recv_buf, 4);
        t1_x3 = esp_timer_get_time();
        t_x3 = t1_x3 - t0_x3;
        t0_x3 = t1_x3;
        sendBuffer.sensorData.x3 = x3.value;
        sendBuffer.sensorData.dataValid = (t_x3 <= UDP_PACKET_DELAY_THRESHOLD) ? (sendBuffer.sensorData.dataValid | (1 << X3_DATA_VALID_BIT)) : (sendBuffer.sensorData.dataValid & ~(1 << X3_DATA_VALID_BIT));
        ESP_LOGI(TCP_TAG, "%lld", t_x3);
    }
}

static void tcp_client_task(void *pvParameters)
{
    int socket_retry_count = 0;

    while (socket_retry_count < MAX_SOCKET_RETRIES)
    {
        // Configure the server information
        struct sockaddr_in dest_addr1;
        dest_addr1.sin_addr.s_addr = inet_addr(DEVICE_1_IP);
        dest_addr1.sin_family = AF_INET;
        dest_addr1.sin_port = htons(DEVICE_PORT);

        // dest_addr2.sin_addr.s_addr = inet_addr(DEVICE_2_IP);
        // dest_addr2.sin_family = AF_INET;
        // dest_addr2.sin_port = htons(DEVICE_PORT);

        // Create the sockets
        int sock1 = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        // int sock2 = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (sock1 < 0) 
        {
            ESP_LOGE(TCP_TAG, "Unable to create socket: errno %d, sock1: %d", errno, sock1);
            break;
        }
        ESP_LOGI(TCP_TAG, "Sockets created");

        // Set socket option to AC_VO for a reduction in latency
        // const int ip_precedence_vi = 6;
        // const int ip_precedence_offset = 7;
        // int priority = (ip_precedence_vi << ip_precedence_offset);
        // setsockopt(sock1, IPPROTO_IP, IP_TOS, &priority, sizeof(priority));
        // setsockopt(sock2, IPPROTO_IP, IP_TOS, &priority, sizeof(priority));

        // Bind the sockets
        // int err = bind(sock1, (struct sockaddr *)&dest_addr1, sizeof(dest_addr1));
        // if (err < 0) 
        // {
        //     ESP_LOGE(TCP_TAG, "Socket unable to bind socket 1: errno %d", errno);
        // }
        // err = bind(sock2, (struct sockaddr *)&dest_addr2, sizeof(dest_addr2));
        // if (err < 0) 
        // {
        //     ESP_LOGE(TCP_TAG, "Socket unable to bind socket 2: errno %d", errno);
        // }
        ESP_LOGI(TCP_TAG, "Socket bound.");

        vTaskDelay(5000 / portTICK_PERIOD_MS);

        ESP_LOGI(TCP_TAG, "Trying to connect..");
        // Connect to the servers
        while(0 != connect(sock1, (struct sockaddr *)&dest_addr1, sizeof(struct sockaddr_in)))
        {
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            ESP_LOGE(TCP_TAG, "Socket unable to connect to dest 1: errno %d, trying again..", errno);
        }
        // while(0 != connect(sock2, (struct sockaddr *)&dest_addr2, sizeof(dest_addr2)))
        // {
        //     vTaskDelay(2000 / portTICK_PERIOD_MS);
        //     ESP_LOGE(TCP_TAG, "Socket unable to connect to dest 2: errno %d, trying again..", errno);
        // }
        ESP_LOGI(TCP_TAG, "Successfully connected");

        t0_x2 = esp_timer_get_time();
        t0_x3 = esp_timer_get_time();

        // initialize the send buffer (struct that will be updated to be sent over)
        sendBuffer.sensorData.x2 = 0.0F;
        sendBuffer.sensorData.x3 = 0.0F;
        sendBuffer.sensorData.dataValid = 0U;

        while (1) 
        {
            char rx_buffer[7] = "";
            (void)zero_set_request;

            // Send a request for the position to device 1
            int err = send(sock1, (const char *)&position_request, sizeof(uint16_t), 0);
            if (err < 0) {
                ESP_LOGE(TCP_TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            int len = recv(sock1, rx_buffer, sizeof(rx_buffer), 0);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TCP_TAG, "recv failed: errno %d", errno);
                break;
            }
            else
            {
                process_data((unsigned char *)rx_buffer, (uint32_t)dest_addr1.sin_addr.s_addr);
            }

            // Send a request for the position to device 2
            // err = send(sock2, (const char *)&position_request, sizeof(uint16_t), 0);
            // if (err < 0) {
            //     ESP_LOGE(TCP_TAG, "Error occurred during sending: errno %d", errno);
            //     break;
            // }
            // len = recv(sock2, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // // Error occurred during receiving
            // if (len < 0) {
            //     ESP_LOGE(TCP_TAG, "recv failed: errno %d", errno);
            //     break;
            // }
            // else
            // {
            //     process_data((unsigned char *)rx_buffer, (uint32_t)dest_addr2.sin_addr.s_addr);
            // }

            // vTaskDelay(0);
        }

        if (sock1 != -1) 
        {
            ESP_LOGE(TCP_TAG, "Shutting down socket and restarting...");
            shutdown(sock1, 0);
            close(sock1);
        }

        socket_retry_count++;
    }
    vTaskDelete(NULL);
}

static void spi_comms_task(void *pvParameters)
{
    esp_err_t ret;
     //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK
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

    //Initialize SPI slave interface
    ret=spi_slave_initialize(HSPI_HOST, &buscfg, &slvcfg, 0);
    assert(ret==ESP_OK);

    uint8_t recvbuf[9] = {0x0000};
    // memset(recvbuf, 0, 33);
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    while(1) 
    {
        //Clear receive buffer, set send buffer to something sane
        memset(recvbuf, 0x00, 9);

        //Set up a transaction of 128 bytes to send/receive
        t.length = 9*8;
        t.tx_buffer = (const void *)sendBuffer.bytes;
        t.rx_buffer = &recvbuf;

        ret = spi_slave_transmit(HSPI_HOST, &t, portMAX_DELAY);
        vTaskDelay(0);
    }

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

    ESP_LOGI(WIFI_TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();

    xTaskCreate(tcp_client_task, "tcp_server", 4096, NULL, 6, NULL);
    xTaskCreate(spi_comms_task, "spi_comms", 4096, NULL, 5, NULL);
}
