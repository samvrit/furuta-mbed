#include "mbed.h"

#define PI 3.14F
#define MICROSECOND 0.000001F

#define CURRENT_LPF_CUTOFF_FREQ_HZ 200U
#define CURRENT_SENSE_OFFSET 0.15F
#define MAX_CURRENT_SENSE 7.6F
#define MOTOR_CONSTANT_KT 0.2525F

#define KP 6.57F
#define KI 11503.0F

#define DUTY_CYCLE_LOWER_BOUND 0.01F
#define DUTY_CYCLE_UPPER_BOUND 0.99F

#define SATURATE(input, lower_limit, upper_limit) ((input) > (upper_limit) ? (upper_limit) : ((input) < (lower_limit) ? (lower_limit) : (input)))
#define LOW_PASS_FILTER(output, input, dt, cutoff_freq) ((output) += ((input) - (output)) * 2 * PI * (cutoff_freq) * (dt) * MICROSECOND)

DigitalOut led(LED1);
DigitalOut motor_enable(PA_8);
PwmOut motor_pwm(PA_5);
DigitalOut inA(PA_6);
DigitalOut inB(PA_7);
AnalogIn currentSense(PA_4);

InterruptIn button(BUTTON1);

Timer t;

RawSerial torque_input(PA_0, PA_1);

volatile bool motor_enabled = false;
volatile bool torqueCommandAvailable = false;
volatile char rx_buffer[sizeof(float) + 1];
volatile int rx_count;

volatile union uart_packet_t {
    float torqueCommand;
    char packet[sizeof(float)];
} uart_packet;

void flip(void)
{
    motor_enable = !motor_enable;
    motor_enabled = !motor_enabled;
}

void rx_irq(void)
{
    led = !led;
    while(torque_input.readable())
    {
        rx_buffer[rx_count] = torque_input.getc();
        if(rx_buffer[rx_count] == '\r')
        {
            torqueCommandAvailable = true;
            rx_count = 0;
            break;
        }
        else
        {
            rx_count++;
        }
    }
    
}


int main()
{
    torque_input.baud(115200);
    torque_input.attach(&rx_irq, RawSerial::RxIrq);

    torque_input.printf("Hello World!\n");

    button.rise(&flip);
    inA = 1;
    inB = 0;

    float duty_cycle = 0.0;
    float currentSenseLPF = 0.0;
    float currentSenseRaw = 0.0;
    float torqueCommand = 0.02;
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
            torque_input.printf("Torque command available! %02X %02X %02X %02X \n", rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]);
            for(int i = 0; i <= 4; i++)
            {
                uart_packet.packet[i] = rx_buffer[i];
            }
            torque_input.printf("New torque command: %f\n", uart_packet.torqueCommand);
            torqueCommand = uart_packet.torqueCommand;
            torqueCommandAvailable = false;
        }
            

        if(motor_enabled)
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

            inA = torqueCommand < 0 ? 0 : 1;
            inB = !inA;
            motor_pwm.write(duty_cycle);    // set duty cycle
        }
        
        dt = t.read_us();

        printf("%d,%f,%f,%f\r\n", dt, currentSenseLPF, torqueFeedback, duty_cycle);

        t.reset();
    }

    return 0; 
}