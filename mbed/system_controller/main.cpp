#include "mbed.h"
#include "acquire_feedback.h"
#include "AMT203.h"
#include "kalman_filter.h"
#include "rtos.h"
#include "arm_math.h"
#include <string>

Thread network_thread;

SPI encoder_spi(D11, D12, D13); // MOSI, MISO, CLK
DigitalOut encoder_cs(D10); // chip select for encoder
AMT203 encoder(&encoder_spi, &encoder_cs);

CAN can1(PA_11, PA_12, 1000000);

Timer timer;
Ticker ticker;
int dt = 0;

typedef union {
    float value;
    char buffer[sizeof(float)];
} canPacket_t;

volatile canPacket_t canPacket;


int main()
{
    
    canPacket.value = -0.5;

    network_thread.start(callback(sensors_receive));

    ThisThread::sleep_for(2000);

    matrices_init();

    printf("Sending signal: %02X %02X %02X %02X\r\n", canPacket.buffer[0], canPacket.buffer[1], canPacket.buffer[2], canPacket.buffer[3]);
    can1.write(CANMessage(0x1, (const char *)&canPacket.buffer, sizeof(float)));

    while (1)
    {
        timer.start();

        float position = encoder.get_position_minus_pi_to_plus_pi();

        printf("Position: %.5f rad\r\n", position);
        
        ThisThread::sleep_for(10);

        osEvent evt = queue.get(0);
        if (evt.status == osEventMessage)
        {
            timer.stop();
            dt = timer.read_us();
            timer.reset();
            udp_frame *frame = (udp_frame *)evt.value.p;
            printf("\n%d | Received: %d, %d, %d\r\n", dt, frame->pos_rad, frame->vel_sign, frame->vel_rad);

            mpool.free(frame);
        }
    }

    return 0;
}