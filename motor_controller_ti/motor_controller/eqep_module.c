/*
 * eqep_module.c
 *
 *  Created on: Apr 9, 2020
 *      Author: Samvrit Srinivas
 */

#include "eqep_module.h"

void FreqCal_calculate(FreqCal_Object *p)
{
    uint32_t temp;
    /* Frequency calculation using eQEP capture counter */
    if((EQEP_getStatus(EQEP1_BASE) & EQEP_STS_UNIT_POS_EVNT) != 0)
    {
        // No capture overflow
        if((EQEP_getStatus(EQEP1_BASE) & EQEP_STS_CAP_OVRFLW_ERROR) == 0)
        {
            temp = (uint32_t)EQEP_getCapturePeriodLatch(EQEP1_BASE);
        }
        else
        {
            temp = 0xFFFF;  // Capture overflow, saturate the result
        }

        // p->freqPR = X / [(t2 - t1) * 10kHz]
        p->freqPR = _IQdiv(p->freqScalerPR, temp);
        temp = p->freqPR;

        if(temp > _IQ(1))
        {
            p->freqPR = _IQ(1);
        }
        else
        {
            p->freqPR = temp;
        }

        // Q0 = Q0 * GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
        // p->freqHzPR = (p->freqPR) * 10kHz = X / (t2 - t1)

        p->freqHzPR = _IQmpy(p->baseFreq, p->freqPR);
        p->PR_calc_count++;
        if(p->PR_calc_count >= 2)
        {
            p->task_complete_PR = 1;
        }

        //
        // Clear unit position event flag and overflow error flag
        //
        EQEP_clearStatus(EQEP1_BASE, (EQEP_STS_UNIT_POS_EVNT | EQEP_STS_CAP_OVRFLW_ERROR));
    }
}





