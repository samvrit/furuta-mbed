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


/*==================DEFINES==================*/
#define SATURATE(input, lower_limit, upper_limit) ((input) > (upper_limit) ? (upper_limit) : ((input) < (lower_limit) ? (lower_limit) : (input)))
#define LOW_PASS_FILTER(output, input, lpf_constant) ((output) += (((input) - (output)) * (lpf_constant)))
#define ABS(input) ((input) < 0 ? -(input) : (input))

#ifdef MOTOR_CHARACTERIZATION
#define CONTROL_CYCLE_TIME_US   800000U // microseconds
#else
#define CONTROL_CYCLE_TIME_US   500U // microseconds
#endif //MOTOR_CHARACTERIZATION

#define EPWM1_TIMER_TBPRD       1000U
#define PI                      3.1415F




/*==================CLA VARIABLES==================*/
typedef struct {
    uint16_t enable;
    float currentAmperes;
    float torqueCommand;
} claInputs_S;
typedef struct {
    uint16_t CMPA;
    uint16_t direction;
} claOutputs_S;

extern claInputs_S claInputs;
extern claOutputs_S claOutputs;

/*==================FUNCTION PROTOTYPES==================*/
__interrupt void Cla1Task1();
__interrupt void Cla1Task8();


#endif //end of _CLA_ASIN_SHARED_H_ definition

//
// End of file
//
