#include "mbed.h"
#include "OdinWiFiInterface.h"
#include <string>

const double alpha_pos = 0.1;
const double alpha_vel = 0.1;

struct udp_frame
{
    unsigned short pos_rad;
    uint8_t vel_sign;
    unsigned int vel_rad;
};

extern MemoryPool<udp_frame, 12> mpool;
extern Queue<udp_frame, 12> queue;

int low_pass_filter(struct udp_frame *frame, struct udp_frame *previous_frame);

int deserialize_frame(unsigned char *buffer, struct udp_frame *frame);

void process_data(unsigned char *recv_buf, nsapi_addr_t *address);

void sensors_receive();