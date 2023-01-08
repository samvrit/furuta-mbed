#include "espnow_receive_callback.h"
#include "wifi_init.h"
#include "queues_and_semaphores.h"

#define TICKS_TO_WAIT (512)

#if (CONFIG_RECEIVER_DEVICE)
bool c2000_host_active = false;
#endif // (CONFIG_RECEIVER_DEVICE)

void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    received_data_S received_data;

    if(len == 5)
    {
        if((data[0] == 0xFF) && (data[1] == 0xFF) && (data[2] == 0xFF) && (data[3] == 0xFF) && (data[4] == 0xFF))
        {
            received_data.received_data_category = RECEIVED_DATA_CALIBRATE_COMMAND;
            xQueueSend(espnow_receive_queue, &received_data, TICKS_TO_WAIT);
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
#if (CONFIG_RECEIVER_DEVICE)
        if(c2000_host_active)
        {
            xQueueSend(espnow_receive_queue, &received_data, TICKS_TO_WAIT);
        }
#else
        xQueueSend(espnow_receive_queue, &received_data, TICKS_TO_WAIT);
#endif // (CONFIG_RECEIVER_DEVICE)
    }
}
