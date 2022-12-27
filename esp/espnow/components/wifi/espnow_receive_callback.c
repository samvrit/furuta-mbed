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

    // timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    // timer_start(TIMER_GROUP_0, TIMER_0);

    // // Wait for 1ms
    // // HW timer peripheral uses 80MHz clock as the base, and the divider is set to 16
    // // Hence, 1ms = 0.001 * (80000000/16) = 5000 counts
    // uint64_t timer_val = 0UL;
    // while(timer_val < 5000UL)
    // {
    //     timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timer_val);
    // }
    // timer_pause(TIMER_GROUP_0, TIMER_0);

    // esp_now_send(peer_mac, angle_to_send.raw, 2U);
}
