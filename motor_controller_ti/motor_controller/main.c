//###########################################################################
//
// FILE:   cla_ex1_asin.c
//
// TITLE:  CLA Arcsine Example for F2837xD.
//
//! \addtogroup driver_example_list
//! <h1>CLA \f$arcsine(x)\f$ using a lookup table (cla_asin_cpu01)</h1>
//!
//! In this example, Task 1 of the CLA will calculate the arcsine of
//! an input argument in the range (-1.0 to 1.0) using a lookup table.
//!
//! \b Memory \b Allocation \n
//!  - CLA1 Math Tables (RAMLS0)
//!    - CLAasinTable - Lookup table
//!  - CLA1 to CPU Message RAM
//!    - fResult - Result of the lookup algorithm
//!  - CPU to CLA1 Message RAM
//!    - fVal - Sample input to the lookup algorithm
//!
//! \b Watch \b Variables \n
//!  - fVal - Argument to task 1
//!  - fResult - Result of \f$arcsin(fVal)\f$
//!
//
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

//
// Included Files
//
#include <cla_shared.h>
#include "driverlib.h"
#include "device.h"


//
// Defines
//
#define WAITSTEP     asm(" RPT #255 || NOP")

#define RESULTS_BUFFER_SIZE     3

//
// Globals
//
uint16_t adcResultRaw;
uint16_t rDataA[4];

typedef union {
    float32_t value;
    uint16_t buffer[sizeof(float32_t)];
} uartPacket_t;

volatile uartPacket_t uartPacket;
volatile float32_t currentSenseA = 0.0;
volatile float32_t torqueCommand = 0.0;
volatile uint16_t dutyCycle = 0U;
//
//Task 1 (C) Variables
// NOTE: Do not initialize the Message RAM variables globally, they will be
// reset during the message ram initialization phase in the CLA memory
// configuration routine
//
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
//
// Function Prototypes
//
void CLA_runTest(void);
void CLA_configClaMemory(void);
void CLA_initCpu1Cla1(void);
void initADC(void);
void initEPWM(void);
void initADCSOC(void);
void initSCIBFIFO(void);
__interrupt void scibRXFIFOISR(void);
__interrupt void adcA1ISR(void);
__interrupt void cla1Isr1();

//
// Main
//
void main(void)
{

    //
    // Intialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pullups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    Interrupt_register(INT_ADCA1, &adcA1ISR);
    Interrupt_register(INT_SCIB_RX, scibRXFIFOISR);

    initSCIBFIFO();

    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_0_EPWM1A);

    //
    // GPIO28 is the SCI Rx pin.
    //
    GPIO_setMasterCore(19, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_19_SCIRXDB);
    GPIO_setDirectionMode(19, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(19, GPIO_QUAL_ASYNC);

    //
    // GPIO18 is the SCI Tx pin.
    //
    GPIO_setMasterCore(18, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_18_SCITXDB);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(18, GPIO_QUAL_ASYNC);

    //
    // Disable sync(Freeze clock to PWM as well)
    //
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    initEPWM();

    //
    // Enable sync and clock to PWM
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    initADC();
    initADCSOC();

    adcResultRaw = 0.0;
    errorIntegral = 0.0;
    claInputs.currentAmperes = 0.0;
    claInputs.torqueCommand = 0.0;
    //
    // Configure the CLA memory spaces first followed by
    // the CLA task vectors
    //
    CLA_configClaMemory();
    CLA_initCpu1Cla1();

    Interrupt_enable(INT_ADCA1);
    Interrupt_enable(INT_SCIB_RX);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    for(;;)
    {
        CLA_runTest();
    }
}

//
// CLA_runTest - Execute CLA task tests for specified vectors
//
void CLA_runTest(void)
{
    claInputs.currentAmperes = currentSenseA;
    claInputs.torqueCommand = torqueCommand;
    CLA_forceTasks(CLA1_BASE,CLA_TASKFLAG_1);
    WAITSTEP;
}

//
// CLA_configClaMemory - Configure CLA memory sections
//
void CLA_configClaMemory(void)
{
    extern uint32_t Cla1funcsRunStart, Cla1funcsLoadStart, Cla1funcsLoadSize;
    EALLOW;

#ifdef _FLASH
    //
    // Copy over code from FLASH to RAM
    //
    memcpy((uint32_t *)&Cla1funcsRunStart, (uint32_t *)&Cla1funcsLoadStart,
           (uint32_t)&Cla1funcsLoadSize);
#endif //_FLASH

    //
    // Initialize and wait for CLA1ToCPUMsgRAM
    //
    MemCfg_initSections(MEMCFG_SECT_MSGCLA1TOCPU);
    while (!MemCfg_getInitStatus(MEMCFG_SECT_MSGCLA1TOCPU)){};

    //
    // Initialize and wait for CPUToCLA1MsgRAM
    //
    MemCfg_initSections(MEMCFG_SECT_MSGCPUTOCLA1);
    while (!MemCfg_getInitStatus(MEMCFG_SECT_MSGCPUTOCLA1)){};

    //
    // Select LS4RAM and LS5RAM to be the programming space for the CLA
    // First configure the CLA to be the master for LS4 and LS5 and then
    // set the space to be a program block
    //
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS4,MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS4,MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS5,MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS5,MEMCFG_CLA_MEM_PROGRAM);

    //
    // Next configure LS0RAM and LS1RAM as data spaces for the CLA
    // First configure the CLA to be the master for LS0(1) and then
    // set the spaces to be code blocks
    //
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS0,MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS0, MEMCFG_CLA_MEM_DATA);

    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS1,MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS1, MEMCFG_CLA_MEM_DATA);

    EDIS;
}

//
// CLA_initCpu1Cla1 - Initialize CLA1 task vectors and end-of-task interrupts
//
void CLA_initCpu1Cla1(void)
{
    //
    // Compute all CLA task vectors
    // On Type-1 CLAs the MVECT registers accept full 16-bit task addresses as
    // opposed to offsets used on older Type-0 CLAs
    //
    EALLOW;
    CLA_mapTaskVector(CLA1_BASE,CLA_MVECT_1,(uint16_t)&Cla1Task1);

    //
    // Enable the IACK instruction to start a task on CLA in software
    // for all  8 CLA tasks. Also, globally enable all 8 tasks (or a
    // subset of tasks) by writing to their respective bits in the
    // MIER register
    //
    CLA_enableIACK(CLA1_BASE);
    CLA_enableTasks(CLA1_BASE, CLA_TASKFLAG_1);

    //
    // Configure the vectors for the end-of-task interrupt for all
    // 8 tasks
    //
    Interrupt_register(INT_CLA1_1, &cla1Isr1);

    //
    // Enable CLA interrupts at the group and subgroup levels
    //
    Interrupt_enable(INT_CLA1_1);
}

//
// Function to configure and power up ADCA.
//
void initADC(void)
{
    //
    // Set ADCCLK divider to /4
    //
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);

    //
    // Set resolution and signal mode (see #defines above) and load
    // corresponding trims.
    //
#if(EX_ADC_RESOLUTION == 12)
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
#elif(EX_ADC_RESOLUTION == 16)
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
#endif
    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);

    //
    // Power up the ADC and then delay for 1 ms
    //
    ADC_enableConverter(ADCA_BASE);
    DEVICE_DELAY_US(1000);
}

//
// Function to configure ePWM1 to generate the SOC.
//
void initEPWM(void)
{

    EPWM_setTimeBasePeriod(EPWM1_BASE, EPWM1_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM1_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0U);

    EPWM_setCounterCompareValue(EPWM1_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                995U);

    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);

    EPWM_disablePhaseShiftLoad(EPWM1_BASE);
    EPWM_setClockPrescaler(EPWM1_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM1_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD);

    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    //
    // Disable SOCA
    //
    EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_A);

    //
    // Configure the SOC to occur on the second up-down-count event
    //
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO_OR_PERIOD);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 2U);


    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
}

//
// Function to configure ADCA's SOC0 to be triggered by ePWM1.
//
void initADCSOC(void)
{
    //
    // Configure SOC0 of ADCA to convert pin A0. The EPWM1SOCA signal will be
    // the trigger.
    //
    // For 12-bit resolution, a sampling window of 15 (75 ns at a 200MHz
    // SYSCLK rate) will be used.  For 16-bit resolution, a sampling window of
    // 64 (320 ns at a 200MHz SYSCLK rate) will be used.
    //
#if(EX_ADC_RESOLUTION == 12)
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA,
                    ADC_CH_ADCIN1, 15);
#elif(EX_ADC_RESOLUTION == 16)
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                    ADC_CH_ADCIN0, 64);
#endif

    //
    // Set SOC0 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);


    ADC_setupPPB(ADCA_BASE, ADC_PPB_NUMBER1, ADC_SOC_NUMBER1);
    ADC_setPPBCalibrationOffset(ADCA_BASE, ADC_PPB_NUMBER1, (int16_t)CURR_SENSE_OFFSET);
}

//
// initSCIAFIFO - Configure SCIA FIFO
//
void initSCIBFIFO()
{
    //
    // 8 char bits, 1 stop bit, no parity. Baud rate is 9600.
    //
    SCI_setConfig(SCIB_BASE, DEVICE_LSPCLK_FREQ, 115200, (SCI_CONFIG_WLEN_8 |
                                                        SCI_CONFIG_STOP_ONE |
                                                        SCI_CONFIG_PAR_NONE));
    SCI_enableModule(SCIB_BASE);
    SCI_resetChannels(SCIB_BASE);
    SCI_enableFIFO(SCIB_BASE);

    //
    // RX and TX FIFO Interrupts Enabled
    //
    //SCI_enableInterrupt(SCIB_BASE, (SCI_INT_RXFF | SCI_INT_TXFF));
    SCI_enableInterrupt(SCIB_BASE, SCI_INT_RXFF);
    SCI_disableInterrupt(SCIB_BASE, SCI_INT_RXERR);

    SCI_setFIFOInterruptLevel(SCIB_BASE, SCI_FIFO_TX4, SCI_FIFO_RX4);
    SCI_performSoftwareReset(SCIB_BASE);

    SCI_resetTxFIFO(SCIB_BASE);
    SCI_resetRxFIFO(SCIB_BASE);

#ifdef AUTOBAUD
    //
    // Perform an autobaud lock.
    // SCI expects an 'a' or 'A' to lock the baud rate.
    //
    SCI_lockAutobaud(SCIB_BASE);
#endif
}

//
// sciaRXFIFOISR - SCIA Receive FIFO ISR
//
__interrupt void scibRXFIFOISR(void)
{

    SCI_readCharArray(SCIB_BASE, rDataA, 4);

    uartPacket.buffer[0] = (rDataA[1] << 8) | rDataA[0];
    uartPacket.buffer[1] = (rDataA[3] << 8) | rDataA[2];

    torqueCommand = uartPacket.value;

    SCI_clearOverflowStatus(SCIB_BASE);

    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_RXFF);

    //
    // Issue PIE ack
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

    Example_PassCount++;
}

//
// ADC A Interrupt 1 ISR
//
__interrupt void adcA1ISR(void)
{
    //
    // Add the latest result to the buffer
    //
    adcResultRaw = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1);

    currentSenseA = ((float)adcResultRaw) * CURR_SENSE_SCALING_FACTOR;

    //
    // Clear the interrupt flag
    //
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    //
    // Check if overflow has occurred
    //
    if(true == ADC_getInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    }

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}


//
// cla1Isr1 - CLA1 ISR 1
//

__interrupt void cla1Isr1 ()
{

    EPWM_setCounterCompareValue(EPWM1_BASE,
                                    EPWM_COUNTER_COMPARE_A,
                                    claOutputs.CMPA);
    //
    // Acknowledge the end-of-task interrupt for task 1
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);

    //
    // Uncomment to halt debugger and stop here
    //
    //asm(" ESTOP0");
}


//
// End of file
//
