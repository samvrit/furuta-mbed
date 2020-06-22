/*
 * interrupt_routines.c
 *
 *  Created on: Jul 3, 2020
 *      Author: Samvrit Srinivas
 */

#include "globals.h"
#include "cla_shared.h"
#include "driverlib.h"
#include "device.h"

#define CURR_SENSE_LPF_CONST                    (2.0F*PI*(1.0F/50000.0F))
#define CURR_SENSE_SCALING_FACTOR_INVERSE       152U
#define CURR_SENSE_SCALING_FACTOR               (1.0F / CURR_SENSE_SCALING_FACTOR_INVERSE)
#define CURR_DEADBAND_UPPER                     40
#define CURR_DEADBAND_LOWER                     -40
#define LV_POWER_ADC_THRESHOLD                  3400U    // ADC connected to 3.3V source

volatile currentVars_S currentVars =
{
    .adcResultRaw = 0U,
    .adcResultRawLPF = 0.0F,
    .adcZeroBias = 0U,
    .currentSenseA = 0.0F,
    .currentSenseALPF = 0.0F
};

typedef struct {
    uint16_t volatile motorEnable : 1;
    uint16_t adcCalibrated : 1;
    uint16_t volatile LV_power_present : 1;
} motorPermissiveFlags_S;

typedef union {
    motorPermissiveFlags_S flags;
    uint16_t all;
} motorPermissives_U;

motorPermissives_U motorPermissives = {
    .flags = {
        .motorEnable = 0U,
        .adcCalibrated = 0U,
        .LV_power_present = 0U
    }
};

// sciaRXFIFOISR - SCIA Receive FIFO ISR
__interrupt void scibRXFIFOISR(void)
{

    SCI_readCharArray(SCIB_BASE, rDataA, 4);

    /* Minimum addressable memory unit in the F28379D is 16 bits as opposed to
     * 8 bits, and size of float is 2 x 16 bits, hence unpack the 8-bit bytes read from SCI accordingly */
    uartPacket.buffer[0] = (rDataA[1] << 8) | rDataA[0];
    uartPacket.buffer[1] = (rDataA[3] << 8) | rDataA[2];

    torqueCommand = uartPacket.value;

#ifdef TEST_MODE
    setMotorPWMFlag = 1U;
#endif // TEST_MODE

    SCI_clearOverflowStatus(SCIB_BASE);

    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_RXFF);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);  // Issue PIE ack
}

// ADC A Interrupt 1 ISR
__interrupt void adcA1ISR(void)
{
    currentVars.adcResultRaw = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1);
    uint16_t lv_power_adc_val = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER4);

    motorPermissives.flags.LV_power_present = (lv_power_adc_val >= LV_POWER_ADC_THRESHOLD);

    int32_t currentSenseZeroCorrected = ((int32_t)currentVars.adcResultRaw - (int32_t)currentVars.adcZeroBias);

    if((currentSenseZeroCorrected <= CURR_DEADBAND_LOWER) || (currentSenseZeroCorrected >= CURR_DEADBAND_UPPER))
    {
        currentVars.currentSenseA = (float32_t)currentSenseZeroCorrected * CURR_SENSE_SCALING_FACTOR;
    }
    else
    {
        currentVars.currentSenseA = 0.0f;
    }

    LOW_PASS_FILTER(currentVars.adcResultRawLPF, (float32_t)currentVars.adcResultRaw, CURR_SENSE_LPF_CONST);
    LOW_PASS_FILTER(currentVars.currentSenseALPF, currentVars.currentSenseA, CURR_SENSE_LPF_CONST);

#ifndef TEST_MODE
    GPIO_writePin(DEBUG_PIN, 1);
    claInputs.currentAmperes = currentVars.currentSenseA;       // set current value
    claInputs.torqueCommand = torqueCommand;                    // set torque command
    claInputs.enable = motorPermissives.all;                    // set enable bit
    CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_1);                  // force CLA task of PI controller
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

    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, motorPermissives.all ? claOutputs.CMPA : EPWM1_TIMER_TBPRD);
    GPIO_writePin(MOTOR_DRIVER_DIRECTION_PIN, claOutputs.direction);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11); // Acknowledge the end-of-task interrupt for task 1
    // asm(" ESTOP0");
}

// cla1Isr1 - CLA1 ISR 8
__interrupt void cla1Isr8 ()
{
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11); // Acknowledge the end-of-task interrupt for task 8
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
        canPacket.buffer[0] = (rDataA[1] << 8) | rDataA[0];
        canPacket.buffer[1] = (rDataA[3] << 8) | rDataA[2];

        torqueCommand = canPacket.value;

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
#ifdef MOTOR_CHARACTERIZATION_MODE
    uartPacket.value = !uartPacket.value;
    setMotorPWMFlag = 1U;
#endif // MOTOR_CHARACTERIZATION_MODE

#ifndef TEST_MODE
    getWiFiDataFlag = 1U;
#endif // TEST_MODE

    getRLSDataFlag = 1U;

    // Acknowledge this interrupt to receive more interrupts from group 1
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

// External interrupt ISR
interrupt void xint1_isr(void)
{
    motorPermissives.flags.motorEnable = !motorPermissives.flags.motorEnable;
    GPIO_writePin(MOTOR_DRIVER_SLEEP_PIN, motorPermissives.flags.motorEnable);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);  // Acknowledge this interrupt to get more from group 1

}

interrupt void xint2_isr(void)
{
    motorDriverFaultCount++;

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);  // Acknowledge this interrupt to get more from group 1

}

// GET FUNCTIONS

uint16_t getMotorEnableFlag(void)
{
    return motorPermissives.flags.motorEnable;
}

uint16_t getADCResultRaw(void)
{
    return currentVars.adcResultRaw;
}

float32_t getADCResultRawLPF(void)
{
    return currentVars.adcResultRawLPF;
}

uint16_t isLVPowerPresent(void)
{
    return motorPermissives.flags.LV_power_present;
}


// SET FUNCTIONS

void setADCCalibratedFlag(uint16_t value)
{
    motorPermissives.flags.adcCalibrated = value;
}

void setADCZeroBias(uint16_t value)
{
    currentVars.adcZeroBias = value;
}



