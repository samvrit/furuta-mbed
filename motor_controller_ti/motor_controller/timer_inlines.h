/*
 * timer_inlines.h
 *
 *  Created on: Jul 13, 2020
 *      Author: Samvrit Srinivas
 */

#ifndef TIMER_INLINES_H_
#define TIMER_INLINES_H_

#include <stdint.h>
#include "cputimer.h"

#define MAX_CPU_TIMER_PRD       0xFFFFFFFF

static inline uint32_t getTimerCounter(void)
{
    return (MAX_CPU_TIMER_PRD - CPUTimer_getTimerCount(CPUTIMER1_BASE));
}

static inline void startTimerCounter(void)
{
    CPUTimer_startTimer(CPUTIMER1_BASE);
}

static inline void stopTimerCounter(void)
{
    CPUTimer_stopTimer(CPUTIMER1_BASE);
}

static inline void resetTimerCounter(void)
{
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
}

static inline void waitCount(uint32_t count)
{
    startTimerCounter();
    while(count >= getTimerCounter()){};
    stopTimerCounter();
}

#endif /* TIMER_INLINES_H_ */
