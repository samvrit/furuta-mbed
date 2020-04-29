//###########################################################################
// $TI Release: F2837xD Support Library v3.09.00.00 $
// $Release Date: Thu Mar 19 07:35:24 IST 2020 $
// $Copyright:
// Copyright (C) 2013-2020 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

/*==================INCLUDES==================*/
#include "cla_shared.h"
#include "peripheral_initializations.h"
#include "IQmathLib.h"
#include "eqep_module.h"

/*==================DEFINES==================*/
#define WAITSTEP     asm(" RPT #255 || NOP")

#define EQEP_BASE_FREQ       10000  // Base/max frequency is 10 kHz
#define FREQ_SCALER_PR  (((DEVICE_SYSCLK_FREQ / 128) * 8) / (2 * EQEP_BASE_FREQ))

/*==================VARIABLES==================*/
int16_t adcPPBResultRaw;
uint16_t adcResultRaw;
float32_t adcResultRawLPF;
uint16_t rDataA[COMM_MSG_RECV_DATA_LENGTH];

FreqCal_Object freq =
{
    FREQ_SCALER_PR,  // freqScalerPR
    EQEP_BASE_FREQ,       // baseFreq
    0, 0, 0, 0    // Initialize outputs to zero
};

typedef union {
    float32_t value;
    uint16_t buffer[sizeof(float32_t)];
} uartPacket_t;

volatile uartPacket_t uartPacket;
volatile float32_t currentSenseA = 0.0; // current sense from ADC, scaled to obtain value in amperes
volatile float32_t torqueCommand = 0.0; // torque command as obtained from SCI
volatile uint32_t CAN_errorFlag = 0;
volatile bool motorEnableFlag = false;

float32_t torqueFeedback;       // tau = Kt * i
float32_t error;                // error signal (torque reference - torque feedback)
float32_t errorInt;             // integral of error signal
float32_t dutyCycle;            // duty cycle of PWM signal
float32_t vbridge;              // bridge voltage that has to be applied across the motor
uint16_t newCMPA, direction;    // outputs for motor driver
uint16_t motorDriverFaultCount; // indicates a fault in the motor driver

/*==================CLA VARIABLE DEFINITIONS==================*/
#ifdef __cplusplus
#pragma DATA_SECTION("CpuToCla1MsgRAM");
claInputs_S claInputs;
#pragma DATA_SECTION("Cla1ToCpuMsgRAM");
claOutputs_S claOutputs;
#else
#pragma DATA_SECTION(claInputs,"CpuToCla1MsgRAM");
claInputs_S claInputs;
#pragma DATA_SECTION(claOutputs,"Cla1ToCpuMsgRAM");
claOutputs_S claOutputs;
#endif //__cplusplus
#ifdef __cplusplus
#pragma DATA_SECTION("CLADataLS0")
#else
#pragma DATA_SECTION(errorIntegral,"CLADataLS0")
#endif //__cplusplus
float errorIntegral;

/*==================FUNCTION PROTOTYPES==================*/
__interrupt void scibRXFIFOISR(void);
__interrupt void adcA1ISR(void);
__interrupt void cla1Isr1();
__interrupt void canISR();
__interrupt void cpuTimer0ISR(void);
interrupt void xint1_isr(void);
interrupt void xint2_isr(void);
void currentControl(void);
void setMotorPWM(void);

/*==================MAIN==================*/
void main(void)
{

    Device_init();              // Intialize device clock and peripherals
    Device_initGPIO();          // Disable pin locks and enable internal pullups.
    Interrupt_initModule();     // Initialize PIE and clear PIE registers. Disables CPU interrupts
    Interrupt_initVectorTable();// Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).

    Interrupt_register(INT_ADCA1, &adcA1ISR);
    Interrupt_register(INT_SCIB_RX, scibRXFIFOISR);
    Interrupt_register(INT_CLA1_1, &cla1Isr1);  // Configure the vectors for the end-of-task interrupt for task 1
    Interrupt_register(INT_CANB0, &canISR);
    Interrupt_register(INT_TIMER0, &cpuTimer0ISR);
    Interrupt_register(INT_XINT1, &xint1_isr);
    Interrupt_register(INT_XINT2, &xint2_isr);

    initSCIBFIFO();
    initCPUTimer();

    configCPUTimer(CPUTIMER0_BASE, CONTROL_CYCLE_TIME_US);
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);

    // GPIO 0 is configured as EPWM1 for duty cycle to the motor driver
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_0_EPWM1A);

    // GPIO 1 is configured as a digital output for motor direction
    GPIO_setPadConfig(MOTOR_DRIVER_DIRECTION_PIN, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(MOTOR_DRIVER_DIRECTION_PIN, GPIO_DIR_MODE_OUT);
    GPIO_setPinConfig(GPIO_1_GPIO1);

    // GPIO 2 is configured as a digital output for motor driver sleep (active LOW, hence configured as open-drain)
    GPIO_setPadConfig(MOTOR_DRIVER_SLEEP_PIN, GPIO_PIN_TYPE_OD);
    GPIO_setDirectionMode(MOTOR_DRIVER_SLEEP_PIN, GPIO_DIR_MODE_OUT);
    GPIO_setPinConfig(GPIO_2_GPIO2);

    // GPIO 3 is configured as a digital input for motor fault indication
    GPIO_setPadConfig(MOTOR_DRIVER_FAULT_IN, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(MOTOR_DRIVER_FAULT_IN, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(MOTOR_DRIVER_FAULT_IN, GPIO_QUAL_SYNC);  // sync to SYSCLKOUT
    GPIO_setInterruptPin(MOTOR_DRIVER_FAULT_IN, GPIO_INT_XINT1);

    // GPIO 12 is configured as CAN TX B
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_12_CANTXB);

    // GPIO 17 is configured as CAN RX B
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_17_CANRXB);

    // GPIO28 is the SCI Rx pin.
    GPIO_setMasterCore(19, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_19_SCIRXDB);
    GPIO_setDirectionMode(19, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(19, GPIO_QUAL_ASYNC);

    // GPIO20 is the EQEP1A for encoder feedback
    GPIO_setPinConfig(GPIO_20_EQEP1A);
    GPIO_setPadConfig(20, GPIO_PIN_TYPE_STD);

    // GPIO18 is the SCI Tx pin.
    GPIO_setMasterCore(18, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_18_SCITXDB);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(18, GPIO_QUAL_ASYNC);

    // GPIO 66 is used as external interrupt for enabling/disabling motor driver
    GPIO_setDirectionMode(66, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(66, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(66, GPIO_QUAL_SYNC);  // sync to SYSCLKOUT
    GPIO_setInterruptPin(66,GPIO_INT_XINT1);

    // GPIO 131 is configured as debug pin for function profiling
    GPIO_setPadConfig(DEBUG_PIN, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(DEBUG_PIN, GPIO_DIR_MODE_OUT);
    GPIO_setPinConfig(GPIO_131_GPIO131);

    GPIO_setInterruptType(GPIO_INT_XINT1, GPIO_INT_TYPE_FALLING_EDGE);
    GPIO_enableInterrupt(GPIO_INT_XINT1);
    GPIO_setInterruptType(GPIO_INT_XINT2, GPIO_INT_TYPE_FALLING_EDGE);
    GPIO_enableInterrupt(GPIO_INT_XINT2);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);  // Disable sync(Freeze clock to PWM as well)
    initEPWM();
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);   // Enable sync and clock to PWM

    initADC();
    initADCSOC();

    // Initialize CANB module
    CAN_initModule(CANB_BASE);
    CAN_setBitRate(CANB_BASE, DEVICE_SYSCLK_FREQ, 1000000, 20);

    CAN_enableInterrupt(CANB_BASE, CAN_INT_IE0 | CAN_INT_ERROR | CAN_INT_STATUS);   // Enable CANB interrupts

    // Initialize variables
    adcResultRaw = 0.0;
    errorIntegral = 0.0;
    motorDriverFaultCount = 0;
    claInputs.enable = 0;
    claInputs.currentAmperes = 0.0;
    claInputs.torqueCommand = 0.0;

    // Configure the CLA memory spaces first followed by the CLA task vectors
    CLA_configClaMemory();
    CLA_initCpu1Cla1();

    // Enable CLA, ADC and SCI interrupts
    Interrupt_enable(INT_CLA1_1);   // Enable CLA interrupts at the group and subgroup levels
    Interrupt_enable(INT_ADCA1);
    Interrupt_enable(INT_SCIB_RX);
    Interrupt_enable(INT_CANB0);
    Interrupt_enable(INT_XINT1);
    Interrupt_enable(INT_XINT2);
    Interrupt_enable(INT_TIMER0);

    CAN_enableGlobalInterrupt(CANB_BASE, CAN_GLOBAL_INT_CANINT0);

    //CANB setup message object
    CAN_setupMessageObject(CANB_BASE, CAN_RX_MSG_OBJ_ID, 0x1, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE, COMM_MSG_RECV_DATA_LENGTH);

    CPUTimer_startTimer(CPUTIMER0_BASE);
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    EINT;
    ERTM;

    initEQEP();

    for(;;)
    {
        GPIO_writePin(MOTOR_DRIVER_SLEEP_PIN, motorEnableFlag);
    }
}

void currentControl(void)
{
    GPIO_writePin(DEBUG_PIN, 1);

    torqueFeedback = MOTOR_CONSTANT_KT * currentSenseA;  // tau = Kt * i
    error = torqueCommand - torqueFeedback;               // compute error signal
    errorInt += error;                                         // integrate error signal
    errorInt = SATURATE(errorInt, -50.0F, 5.00F);

    vbridge = (KP * error) + (KI * errorInt);                  // PI control equation
    vbridge = SATURATE(vbridge, -MOTOR_SUPPLY_VOLTAGE, MOTOR_SUPPLY_VOLTAGE);   // saturate voltage to [-Vmotor, Vmotor]

    dutyCycle = vbridge * PWM_DUTY_CYCLE_SCALER;                    // scale voltage to duty cycle range

    newCMPA = EPWM1_TIMER_TBPRD - (uint16_t)ABS(dutyCycle);         // compute new Compare A register value
    direction = error > 0.0 ? 1 : 0;   // set direction

    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, newCMPA);
    GPIO_writePin(MOTOR_DRIVER_DIRECTION_PIN, direction);

    GPIO_writePin(DEBUG_PIN, 0);
}

void setMotorPWM(void)
{
    dutyCycle = torqueCommand;  // use the same variable for convenience
    newCMPA = EPWM1_TIMER_TBPRD - (uint16_t)dutyCycle;         // compute new Compare A register value
    direction = dutyCycle > 0.0 ? 1 : 0;   // set direction

    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, newCMPA);
    GPIO_writePin(MOTOR_DRIVER_DIRECTION_PIN, direction);
}

// sciaRXFIFOISR - SCIA Receive FIFO ISR
__interrupt void scibRXFIFOISR(void)
{

    SCI_readCharArray(SCIB_BASE, rDataA, 4);

    /* Minimum addressable memory unit in the F28379D is 16 bits as opposed to
     * 8 bits, and size of float is 2 x 16 bits, hence unpack the 8-bit bytes read from SCI accordingly */
    uartPacket.buffer[0] = (rDataA[1] << 8) | rDataA[0];
    uartPacket.buffer[1] = (rDataA[3] << 8) | rDataA[2];

    torqueCommand = uartPacket.value;

#if TEST_MODE
    setMotorPWM();
#endif // TEST_MODE

    SCI_clearOverflowStatus(SCIB_BASE);

    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_RXFF);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);  // Issue PIE ack
}

// ADC A Interrupt 1 ISR
__interrupt void adcA1ISR(void)
{
    adcPPBResultRaw = ADC_readPPBResult(ADCARESULT_BASE, ADC_PPB_NUMBER1);    // Add the latest result to the buffer
    currentSenseA = ((float32_t)adcPPBResultRaw) * CURR_SENSE_SCALING_FACTOR;  // convert ADC reading to amperes by scaling

#if !TEST_MODE
#if USE_CLA
    GPIO_writePin(DEBUG_PIN, 1);
    claInputs.currentAmperes = currentSenseA;
    claInputs.torqueCommand = torqueCommand;
    claInputs.enable = motorEnableFlag;
    CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_1);
#else
    currentControl();
#endif // USE_CLA
#endif // TEST_MODE

    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);   // Clear the interrupt flag

    // Check if overflow has occurred
    if(true == ADC_getInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    }

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);  // Acknowledge the interrupt
}

// cla1Isr1 - CLA1 ISR 1
__interrupt void cla1Isr1 ()
{
    GPIO_writePin(DEBUG_PIN, 0);
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, claOutputs.CMPA);
    GPIO_writePin(MOTOR_DRIVER_DIRECTION_PIN, claOutputs.direction);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11); // Acknowledge the end-of-task interrupt for task 1
    // asm(" ESTOP0");
}

__interrupt void canISR(void)
{
    uint32_t status;

    status = CAN_getInterruptCause(CANB_BASE);  // Read the CAN interrupt status to find the cause of the interrupt

    // If the cause is a controller status interrupt, then get the status
    if(status == CAN_INT_INT0ID_STATUS)
    {
        /* Read the controller status.  This will return a field of status
         * error bits that can indicate various errors.  Error processing
         * is not done in this example for simplicity.  Refer to the
         * API documentation for details about the error status bits.
         * The act of reading this status will clear the interrupt. */
        status = CAN_getStatus(CANB_BASE);

        // Check to see if an error occurred.
        if(((status  & ~(CAN_STATUS_TXOK | CAN_STATUS_RXOK)) != 7) && ((status  & ~(CAN_STATUS_TXOK | CAN_STATUS_RXOK)) != 0))
        {
            CAN_errorFlag = 1;  // Set a flag to indicate some errors may have occurred.
        }
    }

    // Check if the cause is the receive message object 2
    else if(status == CAN_RX_MSG_OBJ_ID)
    {
        CAN_readMessage(CANB_BASE, CAN_RX_MSG_OBJ_ID, rDataA);   // Get the received message

        // Arrange the data to suit the word length of the F28379D
        uartPacket.buffer[0] = (rDataA[1] << 8) | rDataA[0];
        uartPacket.buffer[1] = (rDataA[3] << 8) | rDataA[2];

        torqueCommand = uartPacket.value;

        CAN_clearInterruptStatus(CANB_BASE, CAN_RX_MSG_OBJ_ID);     // Clear the message object interrupt
        CAN_errorFlag = 0;  // Since the message was received, clear any error flags.
    }

    // If something unexpected caused the interrupt, this would handle it.
    else
    {
        // Handle unexpected interrupt
    }

    CAN_clearGlobalInterruptStatus(CANB_BASE, CAN_GLOBAL_INT_CANINT0);  // Clear the global interrupt flag for the CAN interrupt line


    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);  // Acknowledge this interrupt located in group 9
}

// cpuTimer0ISR - Counter for CpuTimer0
__interrupt void cpuTimer0ISR(void)
{

    // Acknowledge this interrupt to receive more interrupts from group 1
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

// External interrupt ISR
interrupt void xint1_isr(void)
{
    motorEnableFlag = !motorEnableFlag;

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);  // Acknowledge this interrupt to get more from group 1

}

interrupt void xint2_isr(void)
{
    motorDriverFaultCount++;

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);  // Acknowledge this interrupt to get more from group 1

}
