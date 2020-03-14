#include "mbed.h"
#include "rtos.h"

PwmOut motor_pwm(PA_5);

DigitalOut inA(PA_6);
DigitalOut inB(PA_7);


int main()
{
    inA = 1;
    inB = 0;
    while(true)
    {
        for(int i = 1; i <= 5; i++)
        {
            float duty_cycle = (float)i*0.1;
            printf("Duty cycle: %.2f\r\n", duty_cycle);
            motor_pwm.period_us(500.0);
            motor_pwm.write(duty_cycle);
            ThisThread::sleep_for(1000);
        }
    }
        
    return 0; 
}