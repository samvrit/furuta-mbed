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
#include "driverlib.h"
#include "device.h"
#include "cla_shared.h"
#include "peripheral_initializations.h"
#include "globals.h"
#include "timer_inlines.h"
#ifndef TEST_MODE
#include "fpu_vector.h"
#endif //TEST_MODE

/*==================DEFINES==================*/
#define RLS_POSITION_SCALER     (2.0F * PI / 16384.0F)

/*==================VARIABLES==================*/

uint16_t rDataA[COMM_MSG_RECV_DATA_LENGTH];
float32_t x1 = 0.0F, x2 = 0.0F, x3 = 0.0F, x4 = 0.0F;
uint16_t wifiDataValid = 1U;
uint64_t wifiDataInvalidCount = 0U;
uint32_t timer_count = 0U;
uint16_t RLS_calibration_complete = 0U;
uint16_t crc = 0U;
uint16_t RLS_position_raw = 0U;
uint16_t RLS_position_status = 0U;

volatile interfacePacket_t uartPacket, canPacket, spiPacket;
volatile float32_t torqueCommand = 0.0F; // torque command as obtained from SCI
volatile uint16_t CAN_errorFlag = 0U;
volatile uint16_t setMotorPWMFlag = 0U;
volatile uint16_t getWiFiDataFlag = 0U;
volatile uint16_t getRLSDataFlag = 0U;

uint16_t motorDriverFaultCount; // indicates a fault in the motor driver


/*==================CLA VARIABLE DEFINITIONS==================*/
#pragma DATA_SECTION(claInputs,"CpuToCla1MsgRAM");
claInputs_S claInputs;
#pragma DATA_SECTION(claOutputs,"Cla1ToCpuMsgRAM");
claOutputs_S claOutputs;

/*==================FUNCTION PROTOTYPES==================*/
void setMotorPWM(float);
void getWiFiData(void);
void RLS_getData(void);
void RLS_triggerSelfCalibration(void);
void RLS_setZeroPositionOffset(uint16_t);


/*==================MAIN==================*/
void main(void)
{

    Device_init();                  // Intialize device clock and peripherals
    Device_initGPIO();              // Disable pin locks and enable internal pullups.
    Interrupt_initModule();         // Initialize PIE and clear PIE registers. Disables CPU interrupts
    Interrupt_initVectorTable();    // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).

    Interrupt_register(INT_ADCA1, &adcA1ISR);
    Interrupt_register(INT_SCIB_RX, &scibRXFIFOISR);
    Interrupt_register(INT_CLA1_1, &cla1Isr1);  // Configure the vectors for the end-of-task interrupt for task 1
    Interrupt_register(INT_CLA1_8, &cla1Isr8);  // Configure the vectors for the end-of-task interrupt for task 8
    Interrupt_register(INT_CANB0, &canISR);
    Interrupt_register(INT_TIMER0, &cpuTimer0ISR);
    Interrupt_register(INT_XINT1, &xint1_isr);  // push button input
    Interrupt_register(INT_XINT2, &xint2_isr);  // motor driver fault indication

    initSCIBFIFO();

    configCPUTimer(CPUTIMER0_BASE, CONTROL_CYCLE_TIME_US, 1);   // timer0 for control cycle
    configCPUTimer(CPUTIMER1_BASE, MAX_CPU_TIMER_PRD, 0);       // timer1 for getting counter value

    configGPIOS();

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);  // Disable sync(Freeze clock to PWM as well)
    initEPWM();
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);   // Enable sync and clock to PWM

    // Initialize CANB module
    CAN_initModule(CANB_BASE);
    CAN_setBitRate(CANB_BASE, DEVICE_SYSCLK_FREQ, 1000000, 20);

    CAN_enableInterrupt(CANB_BASE, CAN_INT_IE0 | CAN_INT_ERROR | CAN_INT_STATUS);   // Enable CANB interrupts

    // Initialize variables
    claInputs.enable = 0;
    claInputs.currentAmperes = 0.0;
    claInputs.torqueCommand = 0.0;

    // Configure the CLA memory spaces first followed by the CLA task vectors
    CLA_configClaMemory();
    CLA_initCpu1Cla();
    waitCount(5U * WAIT_COUNT_1_US); // 5 us to complete initializations in CLA task 8

    initADC();
    initADCSOC();

    // reset the motor driver sleep pin to get it to a known state, since it is an open-drain output
    GPIO_writePin(MOTOR_DRIVER_SLEEP_PIN, 0);
    waitCount(WAIT_COUNT_1_MS);    // 2 s
    GPIO_writePin(MOTOR_DRIVER_SLEEP_PIN, getMotorEnableFlag());

    // Enable CLA, ADC and SCI interrupts
    Interrupt_enable(INT_CLA1_1);
    Interrupt_enable(INT_CLA1_8);
    Interrupt_enable(INT_ADCA1);
    Interrupt_enable(INT_SCIB_RX);
    Interrupt_enable(INT_CANB0);
    Interrupt_enable(INT_XINT1);
    Interrupt_enable(INT_XINT2);
    Interrupt_enable(INT_TIMER0);

    CAN_enableGlobalInterrupt(CANB_BASE, CAN_GLOBAL_INT_CANINT0);

    //CANB setup message object
    CAN_setupMessageObject(CANB_BASE, CAN_RX_MSG_OBJ_ID, 0x1, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE, COMM_MSG_RECV_DATA_LENGTH);

    initSPIA();
    initSPIB();

    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    EINT;
    ERTM;

#ifdef RLS_TRIGGER_SELF_CALIBRATION
    RLS_calibration_complete = 0U;
    RLS_triggerSelfCalibration();
    RLS_calibration_complete = 1U;
#endif // RLS_TRIGGER_SELF_CALIBRATION

    // calibrate ADC zero bias -- this is done because at every startup, the zero bias seems to vary
    setADCCalibratedFlag(0U);                           // set ADC-calibrated flag explicitly to avoid motor getting enabled
    while(!isLVPowerPresent());                         // wait till 3.3V is present so that the current sensor is powered
    waitCount(2U * WAIT_COUNT_1_S);                     // wait for LPF to settle
    setADCZeroBias((uint16_t)getADCResultRawLPF());     // latch the instantaneous zero-current ADC value to the ADC zero bias
    setADCCalibratedFlag(1U);                           // set ADC-calibrated flag to true

    for(;;)
    {
        if(setMotorPWMFlag)
        {
            setMotorPWM(uartPacket.value);
            setMotorPWMFlag = 0U;
        }

        if(getWiFiDataFlag)
        {
            getWiFiData();
            getWiFiDataFlag = 0U;
        }

        if(getRLSDataFlag)
        {
            startTimerCounter();
            RLS_getData();
            timer_count = getTimerCounter();
            resetTimerCounter();
            getRLSDataFlag = 0U;
        }

    }
}

void setMotorPWM(float PWM_dutyCycle)
{
    ASSERT(PWM_dutyCycle <= 1.0F);

    uint32_t PWM_newCMPA = EPWM1_TIMER_TBPRD - (uint16_t)ABS(PWM_dutyCycle * EPWM1_TIMER_TBPRD);         // compute new Compare A register value
    uint32_t motor_direction = PWM_dutyCycle > 0.0 ? 1 : 0;   // set direction

    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, PWM_newCMPA);
    GPIO_writePin(MOTOR_DRIVER_DIRECTION_PIN, motor_direction);
}

void getWiFiData(void)
{
    int i;
    // write 0x00 9 times to transfer 72 bits
#pragma UNROLL(9)
    for(i = 0; i < 9; i++)
    {
        SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0x00);
    }

    while(SPI_getRxFIFOStatus(SPIA_BASE) != SPI_FIFO_RX9){};    // wait until 9 words have been received from wifi device

    // first get x2 by reading from FIFO buffer
    spiPacket.buffer[0] = (SPI_readDataBlockingFIFO(SPIA_BASE) << 8) | SPI_readDataBlockingFIFO(SPIA_BASE);
    spiPacket.buffer[1] = (SPI_readDataBlockingFIFO(SPIA_BASE) << 8) | SPI_readDataBlockingFIFO(SPIA_BASE);
    x2 = spiPacket.value;

    // then get x3
    spiPacket.buffer[0] = (SPI_readDataBlockingFIFO(SPIA_BASE) << 8) | SPI_readDataBlockingFIFO(SPIA_BASE);
    spiPacket.buffer[1] = (SPI_readDataBlockingFIFO(SPIA_BASE) << 8) | SPI_readDataBlockingFIFO(SPIA_BASE);
    x3 = spiPacket.value;

    // in the end, get the byte that indicates if the data is valid or not
    wifiDataValid = SPI_readDataBlockingFIFO(SPIA_BASE);

    if(wifiDataValid != 3U) wifiDataInvalidCount++;

    SPI_resetRxFIFO(SPIA_BASE); // reset receive buffer
}

void RLS_getData(void)
{
    const uint16_t data_request_command[2] = {0x0000, 0x0000};
    int i;

#pragma UNROLL(2)
    for(i = 0; i < 2; i++)
    {
        SPI_writeDataBlockingNonFIFO(SPIB_BASE, data_request_command[i]);
    }

    while(SPI_getRxFIFOStatus(SPIB_BASE) != SPI_FIFO_RX2){};    // wait until 3 words have been received from RLS sensor

    /* reply frame looks like this:
    b24:b10 = position data (14-bit)
    b9:b8 = status bits
    b7:b0 = crc
    */

    // first read position and status
    uint16_t pos_status = SPI_readDataBlockingFIFO(SPIB_BASE);
    RLS_position_raw = (pos_status & 0xFFFC) >> 2;
    RLS_position_status = pos_status & 0x03;
    x1 = ((float)RLS_position_raw) * RLS_POSITION_SCALER;

    // then read crc
    crc = SPI_readDataBlockingFIFO(SPIB_BASE) >> 8;

    SPI_resetRxFIFO(SPIB_BASE); // reset receive buffer
}

void RLS_triggerSelfCalibration(void)
{
    const uint16_t self_calib_sequence[5] = {0xCD00, 0xEF00, 0x8900, 0xAB00, 0x4100};
    int i;

    setMotorPWM(0.05F);  // 20% duty cycle
    waitCount(2U * WAIT_COUNT_1_S);  // 2 s

    SPI_resetTxFIFO(SPIB_BASE);

    for(i = 0; i < 5; i++)
    {
        SPI_writeDataBlockingFIFO(SPIB_BASE, self_calib_sequence[i]);
        waitCount(20U * WAIT_COUNT_1_US);    // 100 us
    }

    waitCount(10U * WAIT_COUNT_1_S);  // 10 s
    setMotorPWM(0.0F);
}

void RLS_setZeroPositionOffset(uint16_t position_offset)
{

}
