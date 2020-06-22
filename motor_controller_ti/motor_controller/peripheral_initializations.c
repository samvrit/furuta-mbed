/*
 * initializations.c
 *
 *  Created on: Apr 8, 2020
 *      Author: Samvrit Srinivas
 */
#include "driverlib.h"
#include "device.h"
#include "peripheral_initializations.h"
#include "cla_shared.h"
#include "globals.h"

void configGPIOS(void)
{
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
    GPIO_setInterruptPin(MOTOR_DRIVER_FAULT_IN, GPIO_INT_XINT2);

    // GPIO 4 is configured as a digital input for motor enable button
    GPIO_setPadConfig(MOTOR_DRIVER_ENABLE, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(MOTOR_DRIVER_ENABLE, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(MOTOR_DRIVER_ENABLE, GPIO_QUAL_SYNC);  // sync to SYSCLKOUT
    GPIO_setInterruptPin(MOTOR_DRIVER_ENABLE, GPIO_INT_XINT1);

    // GPIO11 is the SCI Rx pin.
    GPIO_setMasterCore(11, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_11_SCIRXDB);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(11, GPIO_QUAL_ASYNC);

    // GPIO 12 is configured as CAN TX B
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_12_CANTXB);

    // GPIO 17 is configured as CAN RX B
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_17_CANRXB);

    // GPIO18 is the SCI Tx pin.
    GPIO_setMasterCore(18, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_18_SCITXDB);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(18, GPIO_QUAL_ASYNC);

    // GPIO 131 is configured as debug pin for function profiling
    GPIO_setPadConfig(DEBUG_PIN, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(DEBUG_PIN, GPIO_DIR_MODE_OUT);
    GPIO_setPinConfig(GPIO_131_GPIO131);

    // GPIO59 is the SPISOMIA.
    GPIO_setMasterCore(59, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_59_SPISOMIA);
    GPIO_setPadConfig(59, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(59, GPIO_QUAL_ASYNC);

    // GPIO16 is the SPISIMOA clock pin.
    GPIO_setMasterCore(16, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_16_SPISIMOA);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(16, GPIO_QUAL_ASYNC);

    // GPIO19 is the SPISTEA.
    GPIO_setMasterCore(19, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_19_SPISTEA);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(19, GPIO_QUAL_ASYNC);

    // GPIO60 is the SPICLKA.
    GPIO_setMasterCore(60, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_60_SPICLKA);
    GPIO_setPadConfig(60, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(60, GPIO_QUAL_ASYNC);

    // GPIO64 is the SPISOMIB.
    GPIO_setMasterCore(64, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_64_SPISOMIB);
    GPIO_setPadConfig(64, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(64, GPIO_QUAL_ASYNC);

    // GPIO63 is the SPISIMOB.
    GPIO_setMasterCore(63, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_63_SPISIMOB);
    GPIO_setPadConfig(63, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(63, GPIO_QUAL_ASYNC);

    // GPIO66 is the SPISTEB.
    GPIO_setMasterCore(66, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_66_SPISTEB);
    GPIO_setPadConfig(66, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(66, GPIO_QUAL_ASYNC);

    // GPIO65 is the SPICLKB.
    GPIO_setMasterCore(65, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_65_SPICLKB);
    GPIO_setPadConfig(65, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(65, GPIO_QUAL_ASYNC);

    // setup external interrupt pins
    GPIO_setInterruptType(GPIO_INT_XINT1, GPIO_INT_TYPE_FALLING_EDGE);
    GPIO_enableInterrupt(GPIO_INT_XINT1);
    GPIO_setInterruptType(GPIO_INT_XINT2, GPIO_INT_TYPE_FALLING_EDGE);
    GPIO_enableInterrupt(GPIO_INT_XINT2);
}

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
void CLA_initCpu1Cla(void)
{
    EALLOW;
    CLA_mapTaskVector(CLA1_BASE,CLA_MVECT_1,(uint16_t)&Cla1Task1);  // map task vector to the task to be performed - current controller
    CLA_mapTaskVector(CLA1_BASE,CLA_MVECT_8,(uint16_t)&Cla1Task8);  // map task vector to the task to be performed - variable initializations

    /* Enable the IACK instruction to start a task on CLA in software.
       Also, globally enable task 1 and 8 */
    CLA_enableIACK(CLA1_BASE);
    CLA_enableTasks(CLA1_BASE, CLA_TASKFLAG_1| CLA_TASKFLAG_8);
    CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_8);  // this only initializes the variables
}

// Function to configure and power up ADCA.
void initADC(void)
{

    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);   // Set ADCCLK divider to /4

    // Set resolution and signal mode and load corresponding trims.
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);

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
       SYSCLK rate) will be used. */
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN1, 15);
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN4, 15);

    // Set SOC1 to set the interrupt 1 flag. Enable the interrupt and make sure its flag is cleared.
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER4);

    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
}

// initSCIAFIFO - Configure SCIA FIFO
void initSCIBFIFO()
{
    // 8 char bits, 1 stop bit, no parity. Baud rate is 115200.
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

// configCPUTimer - This function initializes the selected timer to the period
void configCPUTimer(uint32_t cpuTimer, uint32_t period, uint16_t interrupt_enable)
{
    uint32_t count = (uint32_t)(DEVICE_SYSCLK_FREQ / 1000000 * period);
    CPUTimer_setPeriod(cpuTimer, count);

    // Set pre-scale counter to divide by 1 (SYSCLKOUT):
    CPUTimer_setPreScaler(cpuTimer, 0);

    // Initializes timer control register. The timer is stopped, reloaded,
    // free run enabled, and interrupt enabled.
    CPUTimer_stopTimer(cpuTimer);
    CPUTimer_reloadTimerCounter(cpuTimer);
    CPUTimer_setEmulationMode(cpuTimer, CPUTIMER_EMULATIONMODE_RUNFREE);

    if(interrupt_enable)
    {
        CPUTimer_enableInterrupt(cpuTimer);
    }
    else
    {
        CPUTimer_disableInterrupt(cpuTimer);
    }

    CPUTimer_startTimer(cpuTimer);
}

// Function to configure SPI A in FIFO mode.
void initSPIA()
{
    // Must put SPI into reset before configuring it
    SPI_disableModule(SPIA_BASE);

    // SPI configuration. Use a 12.5MHz SPICLK and 8-bit word size.
    SPI_setConfig(SPIA_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0, SPI_MODE_MASTER, 12500000, 8);
    SPI_disableLoopback(SPIA_BASE);
    SPI_setEmulationMode(SPIA_BASE, SPI_EMULATION_FREE_RUN);
    SPI_resetRxFIFO(SPIA_BASE);
    SPI_enableFIFO(SPIA_BASE);

    // Configuration complete. Enable the module.
    SPI_enableModule(SPIA_BASE);
}

// Function to configure SPI B in FIFO mode.
void initSPIB()
{
    // Must put SPI into reset before configuring it
    SPI_disableModule(SPIB_BASE);

    // SPI configuration. Use a 1.5MHz SPICLK and 16-bit word size.
#ifndef TEST_MODE
    SPI_setConfig(SPIB_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0, SPI_MODE_MASTER, 1500000, 16);
#else
    SPI_setConfig(SPIB_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0, SPI_MODE_MASTER, 1000000, 8);
#endif // TEST_MODE
    SPI_disableLoopback(SPIB_BASE);
    SPI_setEmulationMode(SPIB_BASE, SPI_EMULATION_FREE_RUN);
    SPI_resetRxFIFO(SPIB_BASE);
    SPI_enableFIFO(SPIB_BASE);

    // Configuration complete. Enable the module.
    SPI_enableModule(SPIB_BASE);
}
