#include "espnow_receive_callback.h"
#include "wifi_init.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define TICKS_TO_WAIT (512)

void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    xQueueHandle * espnow_queue_handle = get_espnow_queue_handle();

    received_data_S received_data;

    if(len == 5)
    {
        if((data[0] == 0xFF) && (data[1] == 0xFF) && (data[2] == 0xFF) && (data[3] == 0xFF) && (data[4] == 0xFF))
        {
            received_data.received_data_category = RECEIVED_DATA_CALIBRATE_COMMAND;
            xQueueSend(*espnow_queue_handle, &received_data, TICKS_TO_WAIT);
        }
    }
    else if (len == 2)
    {
        received_data.received_data_category = RECEIVED_DATA_ANGLE;

        for(uint8_t i = 0; i < len; i++)
        {
            received_data.angle.raw[i] = data[i];
        }

        for(uint8_t i = 0; i < 6; i++)
        {
            received_data.mac[i] = mac_addr[i];
        }
        xQueueSend(*espnow_queue_handle, &received_data, TICKS_TO_WAIT);
    }
}
