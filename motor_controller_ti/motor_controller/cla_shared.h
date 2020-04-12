#ifndef _CLA_SHARED_H_
#define _CLA_SHARED_H_
//#############################################################################
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
//#############################################################################

/*==================INCLUDES==================*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==================DEFINES==================*/
#define ADC_RESOLUTION          12
#define EPWM1_TIMER_TBPRD       1000U

#define MOTOR_DRIVER_DIRECTION_PIN  1U
#define MOTOR_DRIVER_SLEEP_PIN      2U

#define PI 3.14F
#define MICROSECOND 0.000001F

#define MOTOR_SUPPLY_VOLTAGE    12.0F
#define PWM_DUTY_CYCLE_SCALER   (EPWM1_TIMER_TBPRD / MOTOR_SUPPLY_VOLTAGE)

#define MOTOR_SPEED_THRESHOLD_HZ            10000U

#define ADC_SCALING_FACTOR                  ((1 << ADC_RESOLUTION) / 3.3F)
#define VOLTAGE_DIVIDER_RATIO               0.67F         // R2 = 6.8kOhms, R1 = 3.3kOhms
#define CURR_SENSE_VOLT_PER_AMP             0.4F          // 400mV per ampere
#define CURR_SENSE_OFFSET_V                 0.5F          // 500mV offset
#define CURR_SENSE_OFFSET                   (CURR_SENSE_OFFSET_V * VOLTAGE_DIVIDER_RATIO * ADC_SCALING_FACTOR)
#define CURR_SENSE_SCALING_FACTOR_INVERSE   (CURR_SENSE_VOLT_PER_AMP * VOLTAGE_DIVIDER_RATIO * ADC_SCALING_FACTOR)
#define CURR_SENSE_SCALING_FACTOR           (1.0F / CURR_SENSE_SCALING_FACTOR_INVERSE)

#define MOTOR_CONSTANT_KT 0.2525F           // Nm/A

#define KP 166.08F                          // proportional gain for PI controller
#define KI 26161.30F                        // integral gain for PI controller

#define CAN_RX_MSG_OBJ_ID               2
#define COMM_MSG_RECV_DATA_LENGTH       4

#define SATURATE(input, lower_limit, upper_limit) ((input) > (upper_limit) ? (upper_limit) : ((input) < (lower_limit) ? (lower_limit) : (input)))
#define LOW_PASS_FILTER(output, input, dt, cutoff_freq) ((output) += (((input) - (output)) * 2 * PI * (cutoff_freq) * (dt) * MICROSECOND))
#define ABS(input) ((input) < 0 ? -(input) : (input))

/*==================CLA VARIABLES==================*/
typedef struct {
    float currentAmperes;
    float torqueCommand;
} claInputs_S;
typedef struct {
    uint16_t CMPA;
    uint16_t direction;
} claOutputs_S;

extern claInputs_S claInputs;
extern claOutputs_S claOutputs;
extern float errorIntegral; // This variable will have to retain its value between iterations, and since CLA cannot declare static variables, this is declared as a shared variable

/*==================FUNCTION PROTOTYPES==================*/
__interrupt void Cla1Task1();

#ifdef __cplusplus
}
#endif // extern "C"

#endif //end of _CLA_ASIN_SHARED_H_ definition

//
// End of file
//
