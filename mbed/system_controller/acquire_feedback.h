#include "mbed.h"
#include "OdinWiFiInterface.h"
#include <string>

typedef union {
    float value;
    char buffer[sizeof(float)];
} udpPacket_t;

typedef struct { float x[3]; } state_vector;

extern MemoryPool<state_vector, 16> mpool;
extern Queue<state_vector, 16> feedback_queue;

void sensors_receive();