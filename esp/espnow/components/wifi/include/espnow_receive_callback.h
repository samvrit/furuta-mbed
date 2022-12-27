#ifndef ESPNOW_RECEIVE_CALLBACK_H
#define ESPNOW_RECEIVE_CALLBACK_H

#include <stdint.h>

typedef union
{
    uint16_t value;
    uint8_t raw[2];
} angle_value_to_raw_U;

typedef enum
{
    RECEIVED_DATA_CALIBRATE_COMMAND,
    RECEIVED_DATA_ANGLE
} received_data_category_E;

typedef struct
{
    received_data_category_E received_data_category;
    uint8_t mac[6];
    angle_value_to_raw_U angle;
} received_data_S;

void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len);

#endif // ESPNOW_RECEIVE_CALLBACK_H
