#include "mbed.h"
#include "OdinWiFiInterface.h"
#include <string>

struct udp_frame
{
    unsigned short pos_rad;
    uint8_t vel_sign;
    unsigned int vel_rad;
};

typedef union {
    float value;
    char buffer[sizeof(float)];
} udpPacket_t;

float get_x2(void);
float get_x3(void);

void sensors_receive();