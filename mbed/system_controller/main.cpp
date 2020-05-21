#include "mbed.h"
#include "acquire_feedback.h"
#include "kalman_filter.h"
#include "rtos.h"
#include "arm_math.h"
#include <string>

Thread sensors_thread(priority = osPriorityHigh);
Thread communication_thread(priority = osPriorityHigh1);
Thread controls_thread(priority = osPriorityRealtime);

SPI encoder_spi(D11, D12, D13); // MOSI, MISO, CLK
DigitalOut encoder_cs(D10); // chip select for encoder

CAN can1(PA_11, PA_12, 1000000);

Timer timer;
Ticker ticker_20kHz;
Ticker ticker_10kHz;
int dt = 0;

typedef union {
    float value;
    char buffer[sizeof(float)];
} canPacket_t;

volatile canPacket_t canPacket;

float x1_val = 0.0f, x2_val = 0.0f, x3_val = 0.0f;

void get_state(void)
{
    x2_val = get_x2();
    x3_val = get_x3();
}


int main()
{
    
    canPacket.value = -0.5;
    ticker.attach_us(&get_state, 20);

    sensors_thread.start(callback(sensors_receive));

    ThisThread::sleep_for(2000);

    matrices_init();

    printf("Sending signal: %02X %02X %02X %02X\r\n", canPacket.buffer[0], canPacket.buffer[1], canPacket.buffer[2], canPacket.buffer[3]);
    can1.write(CANMessage(0x1, (const char *)&canPacket.buffer, sizeof(float)));

    timer.start();
    while (1)
    {
        dt = timer.read_us();
        if(dt >= 100)
        {
            x1_val = encoder.get_position_minus_pi_to_plus_pi();
            timer.reset();
            printf("%d | %.5f | %.5f | %.5f\r\n", dt, x1_val, x2_val, x3_val);
        }
    }

    return 0;
}