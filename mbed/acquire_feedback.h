#include "mbed.h"
#include "OdinWiFiInterface.h"
#include <string>

struct udp_frame
{
    unsigned int pos_rad;
    uint8_t vel_sign;
    unsigned int vel_rad;
};

extern MemoryPool<udp_frame, 12> mpool;
extern Queue<udp_frame, 12> queue;

int deserialize_frame(unsigned char *buffer, struct udp_frame *frame);

void sensors_receive();