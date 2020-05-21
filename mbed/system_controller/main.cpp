#include "mbed.h"
#include "acquire_feedback.h"
#include "can_communications.h"
#include "kalman_filter.h"
#include "rtos.h"
#include "arm_math.h"
#include <string>

Thread sensors_thread(osPriorityHigh);
Thread can_communication_thread(osPriorityHigh1);
Thread controls_thread(osPriorityRealtime);

SPI encoder_spi(D11, D12, D13); // MOSI, MISO, CLK
DigitalOut encoder_cs(D10); // chip select for encoder

Timer timer;
Ticker ticker_20kHz;
Ticker ticker_10kHz;

int dt = 0;

void task_20kHz(void)
{

}

void task_10kHz(void)
{
    can_queue.call(can_send);
}

int main()
{
    sensors_thread.start(callback(sensors_receive));

    ThisThread::sleep_for(2000);

    can_communication_thread.start(callback(&can_queue, &EventQueue::dispatch_forever));

    ticker_20kHz.attach_us(task_20kHz, 50);
    ticker_10kHz.attach_us(task_10kHz, 100);

    matrices_init();

    while (1)
    {
        
    }

    return 0;
}