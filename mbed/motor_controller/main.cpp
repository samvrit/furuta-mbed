#include "mbed.h"
#include "rtos.h"

#define LPF_GAIN 0.3F
#define CURRENT_SENSE_OFFSET 0.15F
#define MAX_CURRENT_SENSE 7.6F

PwmOut motor_pwm(PA_5);
DigitalOut inA(PA_7);
DigitalOut inB(PA_8);
AnalogIn currentSense(PA_0);

InterruptIn button(BUTTON1);

Timer t;

void flip(void)
{
    inA = !inA;
    inB = 0;
}

void apply_lpf(float * output, float * input)
{

    *output += (*input - *output) * LPF_GAIN;
}


int main()
{
    button.rise(&flip);
    inA = 0;
    inB = 0;

    float duty_cycle = 0.9;
    float currentSenseLPF = 0.0;
    float currentSenseRaw = 0.0;

    motor_pwm.period_us(500.0);
    
    while(true)
    {
        motor_pwm.write(duty_cycle);

        currentSenseRaw = currentSense.read();

        currentSenseRaw -= CURRENT_SENSE_OFFSET;

        currentSenseRaw *= (MAX_CURRENT_SENSE / (1.0 - CURRENT_SENSE_OFFSET));\
        
        apply_lpf(&currentSenseLPF, &currentSenseRaw);

        printf("%f\r\n", currentSenseLPF);

        t.reset();
    }
        
    return 0; 
}