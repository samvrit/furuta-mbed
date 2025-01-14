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
#include "cla_shared.h"
#ifndef TEST_MODE
#include "fpu_vector.h"
#endif //TEST_MODE

#define KP                              92.42F                         // proportional gain for PI controller
#define KI                              134000.0F                      // integral gain for PI controller
#define MOTOR_CONSTANT_KT               0.258F           // Nm/A
#define MOTOR_SUPPLY_VOLTAGE            12.0F
#define PWM_DUTY_CYCLE_SCALER           (EPWM1_TIMER_TBPRD / MOTOR_SUPPLY_VOLTAGE)
#define TIMESTEP                        (0.00000002F * EPWM1_TIMER_TBPRD)

float torqueFeedback, error, dutyCycle, vbridge;
static float errorIntegral;
uint16_t newCMPA, direction;

/*==================TASKS==================*/
__interrupt void Cla1Task1 ( void )
{
    torqueFeedback = MOTOR_CONSTANT_KT * claInputs.currentAmperes;  // tau = Kt * i
    error = claInputs.torqueCommand - torqueFeedback;               // compute error signal
    errorIntegral += error;                                         // integrate error signal
    errorIntegral = SATURATE(errorIntegral, -50.0F, 50.0F);
    errorIntegral = claInputs.enable ? errorIntegral : 0.0F;

    vbridge = (KP * error) + (KI * errorIntegral * TIMESTEP);                   // PI control equation
    vbridge = SATURATE(vbridge, -MOTOR_SUPPLY_VOLTAGE, MOTOR_SUPPLY_VOLTAGE);   // saturate voltage to [-Vmotor, Vmotor]

    dutyCycle = vbridge * PWM_DUTY_CYCLE_SCALER;                    // scale voltage to duty cycle range

    newCMPA = EPWM1_TIMER_TBPRD - (uint16_t)ABS(dutyCycle);         // compute new Compare A register value
    direction = vbridge > 0.0 ? 1 : 0;   // set direction

    claOutputs.direction = direction;
    claOutputs.CMPA = newCMPA;
}

__interrupt void Cla1Task8 ( void )
{
    torqueFeedback = 0.0F;
    error = 0.0F;
    dutyCycle = 0.0F;
    vbridge = 0.0F;
    errorIntegral = 0.0F;
    newCMPA = EPWM1_TIMER_TBPRD;
    direction = 1U;
}
