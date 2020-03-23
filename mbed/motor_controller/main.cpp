#include "mbed.h"

#define PI 3.14F
#define MICROSECOND 0.000001F

#define CURRENT_LPF_CUTOFF_FREQ_HZ 200U     // cutoff frequency for low pass filter
#define CURRENT_SENSE_OFFSET 0.15F          // ADC value when 0 current is present
#define MAX_CURRENT_SENSE 7.6F              // current value when ADC is at max value
#define MOTOR_CONSTANT_KT 0.2525F           // Nm/A

#define KP 23.16F                           // proportional gain for PID
#define KI 814.62F                          // integral gain for PID

#define DUTY_CYCLE_LOWER_BOUND 0.01F
#define DUTY_CYCLE_UPPER_BOUND 0.99F

#define SATURATE(input, lower_limit, upper_limit) ((input) > (upper_limit) ? (upper_limit) : ((input) < (lower_limit) ? (lower_limit) : (input)))
#define LOW_PASS_FILTER(output, input, dt, cutoff_freq) ((output) += (((input) - (output)) * 2 * PI * (cutoff_freq) * (dt) * MICROSECOND))

DigitalOut motor_enable(PA_8);              // 3.3V power source for gate driver IC
PwmOut motor_pwm(PA_5);                     // PWM for gate driver IC
DigitalOut inA(PA_6);                       // direction pin A
DigitalOut inB(PA_7);                       // direction pin B
AnalogIn currentSense(PA_4);                // current sense analog input

InterruptIn button(BUTTON1);                // button for motor enable/disable

Timer t;                                    // timer for calculating code execution rate

RawSerial torque_input(PA_0, PA_1);         // serial communication for getting torque commands

volatile bool motor_enabled = false;        // flag for when motor is enabled using button
volatile bool torqueCommandAvailable = false;   // flag for when new torque command is available
volatile char rx_buffer[sizeof(float) + 1]; // receive buffer for UART packet containing torque command
volatile int rx_count;                      // index for receive buffer byte array

// use a union to decode float packet received through UART
volatile union uart_packet_t {
    float torqueCommand;
    char packet[sizeof(float)];
} uart_packet;

// Interrupt service routine for when user button is pressed for enabling/disabling motor driver
void flip(void)
{
    motor_enable = !motor_enable;   // 3.3V power source for gate driver IC
    motor_enabled = !motor_enabled;
}

// Interrupt service routine for UART RX
void rx_irq(void)
{
    while(torque_input.readable())
    {
        rx_buffer[rx_count] = torque_input.getc();  // fill available byte into buffer
        if(rx_buffer[rx_count] == '\r')             // look for carriage return
        {
            torqueCommandAvailable = true;
            rx_count = 0;                           // reset buffer index
            break;
        }
        else
        {
            rx_count++;                             // increment index counter
        }
    }
}


int main()
{
    torque_input.baud(115200);                          // set UART baud rate
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

    motor_pwm.period_us(100.0); // set PWM frequency to 100 microseconds (10 kHz)
    
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
            torqueError = abs(torqueCommand) - torqueFeedback;   // compute error signal
            torqueErrorIntegral += torqueError; // compute integral of error signal

            duty_cycle = (KP * torqueError) + (KI * torqueErrorIntegral);   // PI controller
            duty_cycle = SATURATE(duty_cycle, DUTY_CYCLE_LOWER_BOUND, DUTY_CYCLE_UPPER_BOUND);  // saturate duty cycle

            inA = torqueCommand < 0 ? 0 : 1;    // set direction
            inB = !inA;
            motor_pwm.write(duty_cycle);    // set duty cycle
        }
        
        dt = t.read_us();

        printf("%d,%f,%f,%f\r\n", dt, currentSenseLPF, torqueFeedback, duty_cycle);

        t.reset();
    }

    return 0; 
}