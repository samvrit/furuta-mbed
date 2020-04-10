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
uint16_t adcResultRaw;
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
volatile float32_t torqueCommand = 0.0; // torque command as otained from SCI
volatile uint32_t CAN_errorFlag = 0;
volatile bool motorEnableFlag = true;

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
interrupt void xint1_isr(void);

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
    Interrupt_register(INT_XINT1, &xint1_isr);

    initSCIBFIFO();

    // GPIO 0 is configured as EPWM1 for duty cycle to the motor driver
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_0_EPWM1A);

    // GPIO 1 is configured as a digital output for motor direction
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPinConfig(GPIO_1_GPIO1);

    // GPIO 2 is configured as a digital output for motor driver's Vref
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPinConfig(GPIO_2_GPIO2);

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
    GPIO_setQualificationMode(66, GPIO_QUAL_SYNC);  // sync to SYSCLKOUT
    GPIO_setInterruptPin(66,GPIO_INT_XINT1);

    GPIO_setInterruptType(GPIO_INT_XINT1, GPIO_INT_TYPE_FALLING_EDGE);
    GPIO_enableInterrupt(GPIO_INT_XINT1);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);  // Disable sync(Freeze clock to PWM as well)
    initEPWM();
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);   // Enable sync and clock to PWM

    initADC();
    initADCSOC();

    // Initialize CANB module
    CAN_initModule(CANB_BASE);
    CAN_setBitRate(CANB_BASE, DEVICE_SYSCLK_FREQ, 500000, 20);

    CAN_enableInterrupt(CANB_BASE, CAN_INT_IE0 | CAN_INT_ERROR | CAN_INT_STATUS);   // Enable CANB interrupts

    // Initialize CLA variables
    adcResultRaw = 0.0;
    errorIntegral = 0.0;
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

    CAN_enableGlobalInterrupt(CANB_BASE, CAN_GLOBAL_INT_CANINT0);

    //CANB setup message object
    CAN_setupMessageObject(CANB_BASE, CAN_RX_MSG_OBJ_ID, 0x1, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE, COMM_MSG_RECV_DATA_LENGTH);

    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    EINT;
    ERTM;

    for(;;)
    {
        if(motorEnableFlag)
        {
            GPIO_writePin(2, 1);
            CLA_runTask();
        }
        else
        {
            GPIO_writePin(2, 0);
        }

    }
}

void CLA_runTask(void)
{
    claInputs.currentAmperes = currentSenseA;
    claInputs.torqueCommand = torqueCommand;
    CLA_forceTasks(CLA1_BASE,CLA_TASKFLAG_1);   // run task 1 in CLA 1 core
    WAITSTEP;
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

    SCI_clearOverflowStatus(SCIB_BASE);

    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_RXFF);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);  // Issue PIE ack
}

// ADC A Interrupt 1 ISR
__interrupt void adcA1ISR(void)
{

    adcResultRaw = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1);    // Add the latest result to the buffer
    currentSenseA = ((float)adcResultRaw) * CURR_SENSE_SCALING_FACTOR;  // convert ADC reading to amperes by scaling

    FreqCal_calculate(&freq);   // calculate EQEP frequency

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

    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, claOutputs.CMPA);
    GPIO_writePin(1, claOutputs.direction);

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

// External interrupt ISR
interrupt void xint1_isr(void)
{
    motorEnableFlag = !motorEnableFlag;

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);  // Acknowledge this interrupt to get more from group 1

}
