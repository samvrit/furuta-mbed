#include "mbed.h"
#include "acquire_feedback.h"
#include <string>

Thread network_thread;

DigitalOut in_A(PF_1);
DigitalOut in_B(PF_0);
PwmOut motor_pwm(PA_10);

double pwm_dc = 0.0;

Timer timer;
int dt = 0;

int main()
{

    in_A = 1;
    in_B = 0;
    motor_pwm.period_us(500);
    motor_pwm.write(pwm_dc);

    network_thread.start(callback(sensors_receive));

    while (1)
    {
        timer.start();
        osEvent evt = queue.get();
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