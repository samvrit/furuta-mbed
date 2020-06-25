#include "mbed.h"
#include "acquire_feedback.h"
#include "kalman_filter.h"
#include "rtos.h"
#include "arm_math.h"
#include <string>

Thread sensors_thread(osPriorityHigh, OS_STACK_SIZE, nullptr, "sensorsThread");
Thread controls_thread(osPriorityRealtime, OS_STACK_SIZE, nullptr, "controlsThread");

SPI encoder_spi(D11, D12, D13); // MOSI, MISO, CLK
DigitalOut encoder_cs(D10); // chip select for encoder

int dt = 0;

int main()
{
    debug_pin1 = 0;
    sensors_thread.start(callback(sensors_receive));

    ThisThread::sleep_for(2000);

    controls_thread.start(callback(control_loop));

    while (1)
    {
        
    }

    return 0;
}