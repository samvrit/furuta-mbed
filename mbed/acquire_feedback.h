#include "mbed.h"
#include "OdinWiFiInterface.h"
#include <string>

struct udp_frame
{
    unsigned short pos_rad;
    uint8_t vel_sign;
    unsigned int vel_rad;
};

extern MemoryPool<udp_frame, 12> mpool;
extern Queue<udp_frame, 12> queue;

int deserialize_frame(unsigned char *buffer, struct udp_frame *frame);

void process_data(unsigned char *recv_buf, nsapi_addr_t *address);

void sensors_receive();