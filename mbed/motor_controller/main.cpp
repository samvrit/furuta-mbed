/*=======================INCLUDES========================*/
#include "mbed.h"

/*=======================DEFINES========================*/
#define PI 3.14F
#define MICROSECOND 0.000001F

#define CURRENT_LPF_CUTOFF_FREQ_HZ 3500U    // cutoff frequency for low pass filter
#define CURRENT_SENSE_OFFSET 0.091F         // ADC value when 0 current is present
#define CURRENT_SENSE_SCALING_FACTOR 13.73F // calculated by considering rate of change of voltage w.r.t. current, as well as voltage divider circuit (5V -> 3V)
#define MOTOR_CONSTANT_KT 0.2525F           // Nm/A

#define KP 38.93F                           // proportional gain for PI controller
#define KI 1192.6F                          // integral gain for PI controller

#define DUTY_CYCLE_LOWER_BOUND 0.01F
#define DUTY_CYCLE_UPPER_BOUND 0.99F

#define SATURATE(input, lower_limit, upper_limit) ((input) > (upper_limit) ? (upper_limit) : ((input) < (lower_limit) ? (lower_limit) : (input)))
#define LOW_PASS_FILTER(output, input, dt, cutoff_freq) ((output) += (((input) - (output)) * 2 * PI * (cutoff_freq) * (dt) * MICROSECOND))
#define ABS(input) ((input) = (input) < 0 ? -(input) : (input))

/*=======================PERIPHERALS========================*/
DigitalOut motorEnable(PA_8);               // 3.3V power source for gate driver IC
PwmOut motorPWM(PA_5);                      // PWM for gate driver IC
DigitalOut inA(PA_6);                       // direction pin A
DigitalOut inB(PA_7);                       // direction pin B
AnalogIn currentSense(PA_4);                // current sense analog input

InterruptIn buttonPress(BUTTON1);                // button for motor enable/disable

Timer t;                                    // timer for calculating code execution rate

RawSerial torqueInput(PA_0, PA_1);         // serial communication for getting torque commands

/*=======================STRUCTS & UNIONS========================*/
// use a union to decode float packet received through UART
typedef union {
    float value;
    char buffer[sizeof(float)];
} uartPacket_t;

typedef struct {
    bool motorEnabled = false;              // flag for when motor is enabled using button
    bool torqueCommandAvailable = false;    // flag for when new torque command is available
} flags_t;

/*=======================VOLATILES========================*/
volatile char rx_buffer[sizeof(float) + 1]; // receive buffer for UART packet containing torque command
volatile int rx_count;                      // index for receive buffer byte array
volatile flags_t flags;
volatile uartPacket_t uartPacket;

/*=======================INTERRUPT HANDLERS========================*/
// Interrupt service routine for when user button is pressed for enabling/disabling motor driver
void flip(void)
{
    motorEnable = !motorEnable;   // 3.3V power source for gate driver IC
    flags.motorEnabled != flags.motorEnabled;
}

// Interrupt service routine for UART RX
void rx_irq(void)
{
    while(torqueInput.readable())
    {
        rx_buffer[rx_count] = torqueInput.getc();  // fill available byte into buffer
        if(rx_buffer[rx_count] == '\r')             // look for carriage return
        {
            flags.torqueCommandAvailable = true;
            rx_count = 0;                           // reset buffer index
            break;
        }
        else
        {
            rx_count++;                             // increment index counter
        }
    }
}

/*=======================MAIN========================*/
int main()
{
    torqueInput.baud(115200);                          // set UART baud rate
    torqueInput.attach(&rx_irq, RawSerial::RxIrq);

    torqueInput.printf("Hello World!\n");

    buttonPress.rise(&flip);
    inA = 1;
    inB = 0;

    float dutyCycle = 0.0;
    float currentSenseLPF = 0.0;
    float currentSenseRaw = 0.0;
    float torqueCommand = 0.02;
    float torqueFeedback = 0.0;
    float torqueError = 0.0;
    float torqueErrorIntegral = 0.0;
    int dt = 0;

    motorPWM.period_us(100.0); // set PWM frequency to 100 microseconds (10 kHz)
    
    while(true)
    {
        t.start();

        if(flags.torqueCommandAvailable)
        {
            NVIC_DisableIRQ(UART4_IRQn);    // disable UART interrupt while processing new information
            torqueInput.printf("Torque command available! %02X %02X %02X %02X \n", rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]);
            memcpy((void *)&uartPacket.buffer, (void *)&rx_buffer, sizeof(float));  // write buffer contents into union variable
            memset((void *)&rx_buffer, 0, sizeof(float) + 1);   // clear rx buffer
            torqueInput.printf("New torque command: %f\n", uartPacket.value);
            torqueCommand = uartPacket.value;
            flags.torqueCommandAvailable = false;
            NVIC_EnableIRQ(UART4_IRQn);     // enable UART interrupt after processing new torque command
        }
            
        if(flags.motorEnabled)
        {
            currentSenseRaw = (currentSense.read() - CURRENT_SENSE_OFFSET) * CURRENT_SENSE_SCALING_FACTOR; // current sense in amperes
            LOW_PASS_FILTER(currentSenseLPF, currentSenseRaw, dt, CURRENT_LPF_CUTOFF_FREQ_HZ); // apply low pass filter

            torqueFeedback = MOTOR_CONSTANT_KT * currentSenseLPF;   // calculate output torque (tau = Kt * i)
            torqueError = ABS(torqueCommand) - torqueFeedback;   // compute error signal
            torqueErrorIntegral += torqueError; // compute integral of error signal

            dutyCycle = (KP * torqueError) + (KI * torqueErrorIntegral);   // PI controller
            dutyCycle = SATURATE(dutyCycle, DUTY_CYCLE_LOWER_BOUND, DUTY_CYCLE_UPPER_BOUND);  // saturate duty cycle

            inA = torqueCommand < 0 ? 0 : 1;    // set direction
            inB = !inA;
            motorPWM.write(dutyCycle);    // set duty cycle
        }
        
        dt = t.read_us();

        printf("%d,%f,%f,%f\r\n", dt, currentSenseLPF, torqueFeedback, dutyCycle);

        t.reset();
    }

    return 0; 
}