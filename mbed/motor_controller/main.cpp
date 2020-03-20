#include "mbed.h"

#define PI 3.14F
#define MICROSECOND 0.000001F

#define CURRENT_LPF_CUTOFF_FREQ_HZ 200U
#define CURRENT_SENSE_OFFSET 0.15F
#define MAX_CURRENT_SENSE 7.6F
#define MOTOR_CONSTANT_KT 0.2525F

#define KP 6.57F
#define KI 11503.0F

#define DUTY_CYCLE_LOWER_BOUND 0.0F
#define DUTY_CYCLE_UPPER_BOUND 1.0F

#define SATURATE(input, lower_limit, upper_limit) ((input) > (upper_limit) ? (upper_limit) : ((input) < (lower_limit) ? (lower_limit) : (input)))
#define LOW_PASS_FILTER(output, input, dt, cutoff_freq) ((output) += ((input) - (output)) * 2 * PI * (cutoff_freq) * (dt) * MICROSECOND)

DigitalOut led(LED1);
PwmOut motor_pwm(PA_5);
DigitalOut inA(PA_7);
DigitalOut inB(PA_8);
AnalogIn currentSense(PA_0);

InterruptIn button(BUTTON1);

Timer t;

RawSerial torque(PA_0, PA_1);

volatile bool motor_enable = false;
volatile float torqueCommand = 0.02;
volatile bool torqueCommandAvailable = false;
volatile char rx_buffer[5];

void flip(void)
{
    inA = !inA;
    inB = 0;
    motor_enable = !motor_enable;
}

void rx_irq(void)
{
    led = !led;
    int i = 0;
    while(torque.readable())
    {
        rx_buffer[i] = torque.getc();
        if(rx_buffer[i] == '\r')
        {
            torqueCommandAvailable = true;
            break;
        }
        else
        {
            i++;
        }
    }
    
}


int main()
{
    torque.baud(115200);
    torque.attach(&rx_irq, RawSerial::RxIrq);

    torque.printf("Hello World!\n");

    button.rise(&flip);
    inA = 0;
    inB = 0;

    float duty_cycle = 0.0;
    float currentSenseLPF = 0.0;
    float currentSenseRaw = 0.0;
    float torqueFeedback = 0.0;
    float torqueError = 0.0;
    float torqueErrorIntegral = 0.0;
    int dt = 0;

    motor_pwm.period_us(100.0);
    
    while(true)
    {
        t.start();

        if(torqueCommandAvailable)
        {
            torque.printf("Torque command available! %02X %02X %02X %02X \n", rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]);
            torqueCommandAvailable = false;
        }
            

        if(motor_enable)
        {
            currentSenseRaw = currentSense.read();  // read from ADC
            currentSenseRaw -= CURRENT_SENSE_OFFSET;    // adjust for offset
            currentSenseRaw *= (MAX_CURRENT_SENSE / (1.0 - CURRENT_SENSE_OFFSET));  // scale the range
            LOW_PASS_FILTER(currentSenseLPF, currentSenseRaw, dt, CURRENT_LPF_CUTOFF_FREQ_HZ); // apply low pass filter

            torqueFeedback = MOTOR_CONSTANT_KT * currentSenseLPF;   // calculate output torque (tau = Kt * i)
            torqueError = torqueCommand - torqueFeedback;   // compute error signal
            torqueErrorIntegral += torqueError; // compute integral of error signal

            duty_cycle = (KP * torqueError) + (KI * torqueErrorIntegral);   // PI controller
            duty_cycle = SATURATE(duty_cycle, DUTY_CYCLE_LOWER_BOUND, DUTY_CYCLE_UPPER_BOUND);  // saturate duty cycle
            motor_pwm.write(duty_cycle);    // set duty cycle
        }
        
        dt = t.read_us();

        printf("%d,%f,%f,%f\r\n", dt, currentSenseLPF, torqueFeedback, duty_cycle);

        t.reset();
    }

    return 0; 
}