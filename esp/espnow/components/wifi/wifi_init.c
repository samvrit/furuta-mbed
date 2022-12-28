#include "wifi_init.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "esp_now.h"
#include "espnow_receive_callback.h"

#define QUEUE_SIZE (6)

static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

xQueueHandle espnow_queue;

/* WiFi should start before using ESPNOW */
void wifi_init(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR);
}

esp_err_t espnow_init(void)
{
    espnow_queue = xQueueCreate(QUEUE_SIZE, sizeof(received_data_S));
    
    /* Initialize ESPNOW and register sending and receiving callback function. */
    esp_now_init();
    esp_now_register_recv_cb(espnow_recv_cb);

    /* Set primary master key. */
    esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK);

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t peer = {0};
    peer.channel = CONFIG_ESPNOW_CHANNEL;
    peer.ifidx = ESP_IF_WIFI_STA;
    peer.encrypt = false;

    for (uint8_t i = 0; i < ESP_NOW_ETH_ALEN; i++)
    {
        peer.peer_addr[i] = peer_mac[i];
    }

    esp_now_add_peer(&peer);

    return ESP_OK;
}

xQueueHandle * get_espnow_queue_handle(void)
{
    return &espnow_queue;
}
