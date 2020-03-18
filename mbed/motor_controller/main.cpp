#include "mbed.h"
#include "rtos.h"

#define LPF_GAIN 0.3F
#define CURRENT_SENSE_OFFSET 0.15F
#define MAX_CURRENT_SENSE 7.6F
#define MOTOR_CONSTANT_KT 0.2525F

#define KP 0.257F
#define KI 264.82F

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

float saturate(float input, float lower_limit, float upper_limit)
{
    return input > upper_limit ? upper_limit : ( input < lower_limit ? lower_limit : input);
}


int main()
{
    button.rise(&flip);
    inA = 0;
    inB = 0;

    float duty_cycle = 0.9;
    float currentSenseLPF = 0.0;
    float currentSenseRaw = 0.0;
    float torqueFeedback = 0.0;
    float torqueCommand = 0.1;
    float torqueError = 0.0;
    float torqueErrorIntegral = 0.0;

    motor_pwm.period_us(500.0);
    
    while(true)
    {
        currentSenseRaw = currentSense.read();
        currentSenseRaw -= CURRENT_SENSE_OFFSET;
        currentSenseRaw *= (MAX_CURRENT_SENSE / (1.0 - CURRENT_SENSE_OFFSET));
        apply_lpf(&currentSenseLPF, &currentSenseRaw);

        torqueFeedback = currentSenseLPF * MOTOR_CONSTANT_KT;
        torqueError = torqueCommand - torqueFeedback;
        torqueErrorIntegral += torqueError;

        duty_cycle = (KP * torqueError) + (KI * torqueErrorIntegral);
        duty_cycle = saturate(duty_cycle, 0.01, 0.95);
        motor_pwm.write(duty_cycle);

        printf("%f\r\n", currentSenseLPF);

        t.reset();
    }
        
    return 0; 
}