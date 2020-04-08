#ifndef _CLA_ASIN_SHARED_H_
#define _CLA_ASIN_SHARED_H_
//#############################################################################
//
// FILE:   cla_ex1_asin_shared.h
//
// TITLE:  CLA arcsine example header file.
//
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

//
// Included Files
//
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//
// Defines
//

#define PI 3.14F
#define MICROSECOND 0.000001F

#define MOTOR_SUPPLY_VOLTAGE    12.0F

#define GEARBOX_RATIO 30.0F
#define ROTATION_PER_PULSE (2.0F * PI / (16.0F * GEARBOX_RATIO))  // radians per pulse
#define MOTOR_SPEED_THRESHOLD_RAD_S 50000.0F

#define CURRENT_LPF_CUTOFF_FREQ_HZ 3500U    // cutoff frequency for low pass filter
#define CURRENT_SENSE_OFFSET 0.091F

#define ADC_SCALING_FACTOR                  (4095.0F / 3.3F)
#define VOLTAGE_DIVIDER_RATIO               0.67F         // R2 = 6.8kOhms, R1 = 3.3kOhms
#define CURR_SENSE_VOLT_PER_AMP             0.4F          // 400mV per ampere
#define CURR_SENSE_OFFSET_V                 0.5F          // 500mV offset
#define CURR_SENSE_OFFSET                   (CURR_SENSE_OFFSET_V * VOLTAGE_DIVIDER_RATIO * ADC_SCALING_FACTOR)
#define CURR_SENSE_SCALING_FACTOR_INVERSE   (CURR_SENSE_VOLT_PER_AMP * VOLTAGE_DIVIDER_RATIO * ADC_SCALING_FACTOR)
#define CURR_SENSE_SCALING_FACTOR           (1.0F / CURR_SENSE_SCALING_FACTOR_INVERSE)


#define CURRENT_SENSE_SCALING_FACTOR 13.73F // calculated by considering rate of change of voltage w.r.t. current, as well as voltage divider circuit (5V -> 3V)
#define CURRENT_SENSE_LOWER_BOUND 0.0F
#define CURRENT_SENSE_UPPER_BOUND 10.0F
#define MOTOR_CONSTANT_KT 0.2525F           // Nm/A

#define KP 166.08F                          // proportional gain for PI controller
#define KI 26161.30F                        // integral gain for PI controller

#define DUTY_CYCLE_LOWER_BOUND 0.0F
#define DUTY_CYCLE_UPPER_BOUND 1.0F

#define EX_ADC_RESOLUTION       12
#define EPWM1_TIMER_TBPRD       1000U

#define SATURATE(input, lower_limit, upper_limit) ((input) > (upper_limit) ? (upper_limit) : ((input) < (lower_limit) ? (lower_limit) : (input)))
#define LOW_PASS_FILTER(output, input, dt, cutoff_freq) ((output) += (((input) - (output)) * 2 * PI * (cutoff_freq) * (dt) * MICROSECOND))
#define ABS(input) ((input) < 0 ? -(input) : (input))

//
// Globals
//

//
//Task 1 (C) Variables
//
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
extern float y[];                       //Result vector
extern float fVal1, fVal2;              //Holds the input argument to the task
extern float fResult1, fResult2;        //The arsine of the input argument
extern float errorIntegral;

// Function Prototypes
//
// The following are symbols defined in the CLA assembly code
// Including them in the shared header file makes them
// .global and the main CPU can make use of them.
//
__interrupt void Cla1Task1();

#ifdef __cplusplus
}
#endif // extern "C"

#endif //end of _CLA_ASIN_SHARED_H_ definition

//
// End of file
//
