/*
 * eqep_module.h
 *
 *  Created on: Apr 9, 2020
 *      Author: sasrinivas
 */

#ifndef EQEP_MODULE_H_
#define EQEP_MODULE_H_

#include "IQmathLib.h"
#include "driverlib.h"

#define TB_CLK    DEVICE_SYSCLK_FREQ / 2    // Time base clock is SYSCLK / 2
#define PWM_CLK   5000                      // We want to output at 5 kHz
#define PRD_VAL   (TB_CLK / (PWM_CLK * 2))  // Calculate value period value
                                            // for up-down count mode

typedef struct
{
    uint32_t freqScalerPR;  // Parameter: Scaler converting 1/N cycles to a
                            // GLOBAL_Q freq (Q0) - independently with global Q
    uint32_t baseFreq;      // Parameter: Maximum freq

    _iq freqPR;             // Output: Freq in per-unit using capture unit
    int32_t freqHzPR;       // Output: Freq in Hz, measured using Capture unit

    bool task_complete_PR; // Output : calculation is done.
    int16_t PR_calc_count; // Output : calculation is done.
} FreqCal_Object;

typedef FreqCal_Object *FreqCal_Handle;

void FreqCal_calculate(FreqCal_Handle);

#endif /* EQEP_MODULE_H_ */
