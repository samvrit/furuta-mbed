/*
 * globals.h
 *
 *  Created on: Jul 3, 2020
 *      Author: Samvrit Srinivas
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#include <stdint.h>
#include "driverlib/inc/hw_types.h"

#define WAITSTEP     asm(" RPT #255 || NOP")

#define MOTOR_DRIVER_DIRECTION_PIN  1U
#define MOTOR_DRIVER_SLEEP_PIN      2U
#define MOTOR_DRIVER_FAULT_IN       3U
#define MOTOR_DRIVER_ENABLE         4U
#define DEBUG_PIN                   131U

#define WAIT_COUNT_1_S          200000000U  // 200000000/200000000 = 1 s
#define WAIT_COUNT_1_MS         200000U     // 200000/200000000 = 1e-3 s
#define WAIT_COUNT_1_US         200U        // 200/200000000  = 1e-6 s

#define CAN_RX_MSG_OBJ_ID               2
#define COMM_MSG_RECV_DATA_LENGTH       4

typedef union {
    float32_t value;
    uint16_t buffer[sizeof(float32_t)];
} interfacePacket_t;

typedef struct {
    uint16_t adcResultRaw;                      // raw ADC value
    float32_t adcResultRawLPF;                  // raw ADC value passed through an LPF
    uint16_t adcZeroBias;                       // ADC result at zero current
    float32_t currentSenseA;                    // current value in amperes
    float32_t currentSenseALPF;                 // current value passed though an LPF in amperes
} currentVars_S;

extern int16_t adcPPBResultRaw;
extern uint16_t adcResultRaw;
extern float32_t adcResultRawLPF;
extern uint16_t adcMax;
extern uint16_t adcMin;
extern uint16_t rDataA[COMM_MSG_RECV_DATA_LENGTH];

extern volatile interfacePacket_t uartPacket, canPacket, spiPacket;
extern volatile float32_t torqueCommand; // torque command as obtained from SCI
extern volatile uint16_t CAN_errorFlag;
extern volatile uint16_t setMotorPWMFlag;
extern volatile uint16_t getWiFiDataFlag;
extern volatile uint16_t getRLSDataFlag;

extern uint16_t motorDriverFaultCount; // indicates a fault in the motor driver

uint16_t getMotorEnableFlag(void);
uint16_t getADCResultRaw(void);
float32_t getADCResultRawLPF(void);
uint16_t isLVPowerPresent(void);
void setADCCalibratedFlag(uint16_t value);
void setADCZeroBias(uint16_t value);

__interrupt void scibRXFIFOISR(void);
__interrupt void adcA1ISR(void);
__interrupt void cla1Isr1();
__interrupt void cla1Isr8();
__interrupt void canISR();
__interrupt void cpuTimer0ISR(void);
interrupt void xint1_isr(void);
interrupt void xint2_isr(void);


#endif /* GLOBALS_H_ */
