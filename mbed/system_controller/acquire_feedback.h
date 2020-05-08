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

extern MemoryPool<udpPacket_t, 12> mpool;
extern Queue<udpPacket_t, 12> queue;

void process_data(unsigned char *recv_buf, nsapi_addr_t *address);

void sensors_receive();