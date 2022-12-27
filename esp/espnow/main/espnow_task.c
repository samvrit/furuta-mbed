#include "espnow_task.h"
#include "espnow_receive_callback.h"
#include "wifi_init.h"
#include "freertos/semphr.h"

void espnow_task(void *pvParameter)
{
    received_data_S received_data;

    xQueueHandle * espnow_queue_handle = get_espnow_queue_handle();

    for(;;)
    {
        if(xQueueReceive(*espnow_queue_handle, &received_data, portMAX_DELAY) == pdTRUE)
        {
            switch (received_data.received_data_category)
            {
            case RECEIVED_DATA_CALIBRATE_COMMAND:
                {
                    // Calibrate routine
                    break;
                }
                
            case RECEIVED_DATA_ANGLE:
            default:
                {
                    // received angle process
                    break;
                };
            }
        }
    }
}
