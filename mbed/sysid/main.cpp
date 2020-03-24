#include "mbed.h"
#include "PwmIn.h"

#define PI 3.14F
#define ROTATION_PER_PULSE (2.0F * PI / 64.0)

#define ABS(input) ((input) = (input) < 0 ? -(input) : (input))

DigitalOut motor_enable(PA_8);              // 3.3V power source for gate driver IC
PwmOut motor_pwm(PA_5);                     // PWM for gate driver IC
PwmIn encoder(PB_6);                        // encoder input
DigitalOut inA(PA_6);                       // direction pin A
DigitalOut inB(PA_7);                       // direction pin B

InterruptIn button(BUTTON1);                // button for motor enable/disable

Timer t;

RawSerial dutyCycleInput(PA_0, PA_1);         // serial communication for getting duty cycle commands

volatile bool motor_enabled = false;        // flag for when motor is enabled using button
volatile bool dutyCycleCommandAvailable = false; // flag for when new duty cycle command is available
volatile char rx_buffer[sizeof(float) + 1]; // receive buffer for UART packet containing duty cycle command
volatile int rx_count;                      // index for receive buffer byte array

// use a union to decode float packet received through UART
volatile union uart_packet_t {
    float dutyCycleCommand;
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
    while(dutyCycleInput.readable())
    {
        rx_buffer[rx_count] = dutyCycleInput.getc();  // fill available byte into buffer
        if(rx_buffer[rx_count] == '\r')             // look for carriage return
        {
            dutyCycleCommandAvailable = true;
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
    dutyCycleInput.baud(115200);                          // set UART baud rate
    dutyCycleInput.attach(&rx_irq, RawSerial::RxIrq);

    dutyCycleInput.printf("Hello World!\r\n");

    button.rise(&flip);

    inA = 1;
    inB = 0;

    float dutyCycle = 0.0;
    float motorSpeed = 0.0;
    float dt = 0.0;

    motor_pwm.period_us(100.0); // set PWM frequency to 100 microseconds (10 kHz)

    while(true)
    {
        if(dutyCycleCommandAvailable)
        {
            dutyCycleInput.printf("Torque command available! %02X %02X %02X %02X \n", rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]);
            for(int i = 0; i <= 4; i++)
            {
                uart_packet.packet[i] = rx_buffer[i];
            }
            dutyCycleInput.printf("New torque command: %f\n", uart_packet.dutyCycleCommand);
            dutyCycle = uart_packet.dutyCycleCommand;
            dutyCycleCommandAvailable = false;

            inA = dutyCycle < 0 ? 0 : 1;    // set direction
            inB = !inA;
        }

        motor_pwm.write(ABS(dutyCycle));    // set duty cycle

        // obtain speed of motor
        dt = encoder.period();
        motorSpeed = ROTATION_PER_PULSE / dt;   // radians per second

        printf("%f\r\n", motorSpeed);
    }

    return 0;
}