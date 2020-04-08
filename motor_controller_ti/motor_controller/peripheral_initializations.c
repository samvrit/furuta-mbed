/*
 * initializations.c
 *
 *  Created on: Apr 8, 2020
 *      Author: Samvrit Srinivas
 */

#include "peripheral_initializations.h"
#include "cla_shared.h"

// CLA_configClaMemory - Configure CLA memory sections
void CLA_configClaMemory(void)
{
    extern uint32_t Cla1funcsRunStart, Cla1funcsLoadStart, Cla1funcsLoadSize;
    EALLOW;

#ifdef _FLASH
    memcpy((uint32_t *)&Cla1funcsRunStart, (uint32_t *)&Cla1funcsLoadStart, (uint32_t)&Cla1funcsLoadSize); // Copy over code from FLASH to RAM
#endif //_FLASH

    // Initialize and wait for CLA1ToCPUMsgRAM
    MemCfg_initSections(MEMCFG_SECT_MSGCLA1TOCPU);
    while (!MemCfg_getInitStatus(MEMCFG_SECT_MSGCLA1TOCPU)){};

    // Initialize and wait for CPUToCLA1MsgRAM
    MemCfg_initSections(MEMCFG_SECT_MSGCPUTOCLA1);
    while (!MemCfg_getInitStatus(MEMCFG_SECT_MSGCPUTOCLA1)){};

    /* Select LS4RAM and LS5RAM to be the programming space for the CLA
       First configure the CLA to be the master for LS4 and LS5 and then
       set the space to be a program block */
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS4,MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS4,MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS5,MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS5,MEMCFG_CLA_MEM_PROGRAM);

    /* Next configure LS0RAM and LS1RAM as data spaces for the CLA
       First configure the CLA to be the master for LS0(1) and then
       set the spaces to be code blocks */
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS0,MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS0, MEMCFG_CLA_MEM_DATA);

    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS1,MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS1, MEMCFG_CLA_MEM_DATA);

    EDIS;
}

// CLA_initCpu1Cla1 - Initialize CLA1 task vectors and end-of-task interrupts
void CLA_initCpu1Cla1(void)
{
    EALLOW;
    CLA_mapTaskVector(CLA1_BASE,CLA_MVECT_1,(uint16_t)&Cla1Task1);  // map task vector to the task to be performed

    /* Enable the IACK instruction to start a task on CLA in software.
       Also, globally enable task 1 */
    CLA_enableIACK(CLA1_BASE);
    CLA_enableTasks(CLA1_BASE, CLA_TASKFLAG_1);
}

// Function to configure and power up ADCA.
void initADC(void)
{

    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);   // Set ADCCLK divider to /4

    // Set resolution and signal mode and load corresponding trims.
#if(ADC_RESOLUTION == 12)
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
#elif(ADC_RESOLUTION == 16)
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
#endif

    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);    // Set pulse positions to end of conversion

    // Power up the ADC and then delay for 1 ms
    ADC_enableConverter(ADCA_BASE);
    DEVICE_DELAY_US(1000);
}

// Function to configure ePWM1 to generate the SOC.
void initEPWM(void)
{

    EPWM_setTimeBasePeriod(EPWM1_BASE, EPWM1_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM1_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0U);

    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, EPWM1_TIMER_TBPRD); // initialize duty cycle to be 0

    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);

    EPWM_disablePhaseShiftLoad(EPWM1_BASE);
    EPWM_setClockPrescaler(EPWM1_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    EPWM_setCounterCompareShadowLoadMode(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD);

    // Set actions: 1) set output high when counter is counting up and counter = CMPA, 2) set output low when counter is counting down and counter = CMPA
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_A); // Disable SOCA before configuring it

    // Configure the SOC to occur on the second up-down-count event
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO_OR_PERIOD);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 2U);

    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
}

// Function to configure ADCA's SOC1 to be triggered by ePWM1.
void initADCSOC(void)
{
    /* Configure SOC1 of ADCA to convert pin A1. The EPWM1SOCA signal will be
       the trigger.
       For 12-bit resolution, a sampling window of 15 (75 ns at a 200MHz
       SYSCLK rate) will be used.  For 16-bit resolution, a sampling window of
       64 (320 ns at a 200MHz SYSCLK rate) will be used. */
#if(ADC_RESOLUTION == 12)
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN1, 15);
#elif(ADC_RESOLUTION == 16)
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN1, 64);
#endif

    // Set SOC1 to set the interrupt 1 flag. Enable the interrupt and make sure its flag is cleared.
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    // Setup post-processing block 1 to process SOC1 of ADCA and subtract the sensor offset
    ADC_setupPPB(ADCA_BASE, ADC_PPB_NUMBER1, ADC_SOC_NUMBER1);
    ADC_setPPBCalibrationOffset(ADCA_BASE, ADC_PPB_NUMBER1, (int16_t)CURR_SENSE_OFFSET);
}

// initSCIAFIFO - Configure SCIA FIFO
void initSCIBFIFO()
{
    // 8 char bits, 1 stop bit, no parity. Baud rate is 9600.
    SCI_setConfig(SCIB_BASE, DEVICE_LSPCLK_FREQ, 115200, (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE));
    SCI_enableModule(SCIB_BASE);
    SCI_resetChannels(SCIB_BASE);
    SCI_enableFIFO(SCIB_BASE);

    // RX FIFO Interrupt Enabled
    SCI_enableInterrupt(SCIB_BASE, SCI_INT_RXFF);
    SCI_disableInterrupt(SCIB_BASE, SCI_INT_RXERR);

    SCI_setFIFOInterruptLevel(SCIB_BASE, SCI_FIFO_TX4, SCI_FIFO_RX4);   // Setup interrupt after 4 bytes are received in the buffer
    SCI_performSoftwareReset(SCIB_BASE);

    SCI_resetTxFIFO(SCIB_BASE);
    SCI_resetRxFIFO(SCIB_BASE);
}


