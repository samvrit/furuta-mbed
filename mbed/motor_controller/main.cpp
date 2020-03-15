#include "mbed.h"
#include "rtos.h"

PwmOut motor_pwm(PA_5);

DigitalOut inA(PA_7);
DigitalOut inB(PA_8);

AnalogIn currentSense(PA_6);


int main()
{
    inA = 1;
    inB = 0;
    while(true)
    {
        for(int i = 0; i <= 9; i++)
        {
            uint16_t senseVal = currentSense.read_u16();
            float duty_cycle = (float)i*0.1;
            motor_pwm.period_us(500.0);
            motor_pwm.write(duty_cycle);
            printf("Duty cycle: %.2f | Current: %u\r\n", duty_cycle, senseVal);
            ThisThread::sleep_for(1000);
        }
    }
        
    return 0; 
}