/*
 * state_machine.c
 *
 *  Created on: 10-Jan-2023
 *      Author: Samvrit
 */

#include "state_machine.h"

#include <stdbool.h>

#define MAX(a, b) __max((a), (b))
#define MIN(a, b) __min((a), (b))

#define SAT(x, max, min) (MAX(MIN((x), (max)), (min)))

#define PI (3.1415f)
#define DEG2RAD(x) ((x) * (2.0f * PI / 360.0f ))
#define ANGLES_WITHIN_BOUNDS_2DEG (2.0f)
#define ANGLES_WITHIN_BOUNDS_5DEG (5.0f)
#define ANGLES_WITHIN_BOUNDS_10DEG (10.0f)
#define ANGLES_WITHIN_BOUNDS_20DEG (20.0f)
#define ANGLES_WITHIN_BOUNDS_60DEG (60.0f)
#define ANGLES_WITHIN_BOUNDS_180DEG (180.0f)

#define TIMER_1SEC_1KHZ (1000U)
#define TIMER_2_5_SEC_1KHZ (2500U)

uint32_t init_timer = 0U;
uint32_t qualifying_timer = 0U;

static inline bool timer_step(const bool condition, uint32_t threshold, uint32_t * timer_state)
{
    bool timer_output = false;

    if(condition)
    {
        *timer_state += 1U;
    }
    else
    {
        *timer_state = 0U;
    }

    *timer_state = SAT(*timer_state, threshold, 0U);

    timer_output = (*timer_state >= threshold);

    return timer_output;
}

static inline bool check_measurements_within_bounds(const float measurements[3], const float bounds_deg1, const float bounds_deg2, const float bounds_deg3)
{
    const bool check1 = fabsf(measurements[0]) < DEG2RAD(bounds_deg1);
    const bool check2 = fabsf(measurements[1]) < DEG2RAD(bounds_deg2);

#if (SINGLE_PENDULUM)
    const bool check3 = true;
#else
    const bool check3 = fabsf(measurements[2]) < DEG2RAD(bounds_deg3);
#endif

    const bool checks = check1 && check2 && check3;

    return checks;
}

#pragma CODE_SECTION(state_machine_step, ".TI.ramfunc")
controller_state_E state_machine_step(const float measurements[3], const bool fault_present)
{
    static controller_state_E state = CONTROLLER_INIT;

    switch(state)
    {
        case CONTROLLER_INIT:
        {
            const bool init_timer_expired = timer_step(true, TIMER_1SEC_1KHZ, &init_timer);

            if(init_timer_expired)
            {
                init_timer = 0U;
                state = CONTROLLER_STANDBY;
            }

            break;
        }
        case CONTROLLER_STANDBY:
        {
            const bool measurements_within_bounds = check_measurements_within_bounds(measurements, ANGLES_WITHIN_BOUNDS_20DEG, ANGLES_WITHIN_BOUNDS_10DEG, ANGLES_WITHIN_BOUNDS_10DEG);

            if(measurements_within_bounds)
            {
                state = CONTROLLER_QUALIFYING;
            }

            break;
        }
        case CONTROLLER_QUALIFYING:
        {
#if (SINGLE_PENDULUM)
            const bool measurements_within_bounds = check_measurements_within_bounds(measurements, ANGLES_WITHIN_BOUNDS_20DEG, ANGLES_WITHIN_BOUNDS_10DEG, ANGLES_WITHIN_BOUNDS_10DEG);
#else
            const bool measurements_within_bounds = check_measurements_within_bounds(measurements, ANGLES_WITHIN_BOUNDS_10DEG, ANGLES_WITHIN_BOUNDS_5DEG, ANGLES_WITHIN_BOUNDS_5DEG);
#endif

            const bool qualifying_timer_expired = timer_step(measurements_within_bounds, TIMER_2_5_SEC_1KHZ, &qualifying_timer);

            if(!measurements_within_bounds)
            {
                qualifying_timer = 0U;
                state = CONTROLLER_STANDBY;
            }
            else if(qualifying_timer_expired)
            {
                qualifying_timer = 0U;
                state = CONTROLLER_ACTIVE;
            }

            break;
        }
        case CONTROLLER_ACTIVE:
        {
            const bool measurements_within_bounds = check_measurements_within_bounds(measurements, ANGLES_WITHIN_BOUNDS_180DEG, ANGLES_WITHIN_BOUNDS_60DEG, ANGLES_WITHIN_BOUNDS_60DEG);

            if((!measurements_within_bounds) || (fault_present))
            {
                state = CONTROLLER_STANDBY;
            }

            break;
        }
    }

    return state;
}
