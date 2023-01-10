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
#define ANGLES_WITHIN_BOUNDS_60DEG (60.0f)

#define TIMER_1SEC_1KHZ (1000U)
#define TIMER_4SEC_1KHZ (4000U)

uint32_t standby_timer = 0U;
uint32_t qualifying_timer = 0U;

static bool timer_step(const bool condition, uint32_t threshold, uint32_t * timer_state)
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

static bool check_measurements_within_bounds(const float measurements[3], const float bounds_deg1, const float bounds_deg2, const float bounds_deg3)
{
    const bool check1 = fabsf(measurements[0]) < DEG2RAD(bounds_deg1);
    const bool check2 = fabsf(measurements[1]) < DEG2RAD(bounds_deg2);
    const bool check3 = fabsf(measurements[2]) < DEG2RAD(bounds_deg3);

    const bool checks = check1 && check2 && check3;

    return checks;
}

controller_state_E state_machine_step(const float measurements[3])
{
    static controller_state_E state = CONTROLLER_STANDBY;

    switch(state)
    {
        case CONTROLLER_STANDBY:
        {
            const bool standby_timer_expired = timer_step(true, TIMER_1SEC_1KHZ, &standby_timer);

            const bool measurements_within_bounds = check_measurements_within_bounds(measurements, ANGLES_WITHIN_BOUNDS_10DEG, ANGLES_WITHIN_BOUNDS_2DEG, ANGLES_WITHIN_BOUNDS_2DEG);

            if(standby_timer_expired && measurements_within_bounds)
            {
                standby_timer = 0U;
                state = CONTROLLER_QUALIFYING;
            }

            break;
        }
        case CONTROLLER_QUALIFYING:
        {
            const bool measurements_within_bounds = check_measurements_within_bounds(measurements, ANGLES_WITHIN_BOUNDS_10DEG, ANGLES_WITHIN_BOUNDS_2DEG, ANGLES_WITHIN_BOUNDS_2DEG);

            const bool qualifying_timer_expired = timer_step(measurements_within_bounds, TIMER_4SEC_1KHZ, &qualifying_timer);

            if(qualifying_timer_expired)
            {
                qualifying_timer = 0U;
                state = CONTROLLER_ACTIVE;
            }

            break;
        }
        case CONTROLLER_ACTIVE:
        {
            const bool measurements_within_bounds = check_measurements_within_bounds(measurements, ANGLES_WITHIN_BOUNDS_60DEG, ANGLES_WITHIN_BOUNDS_5DEG, ANGLES_WITHIN_BOUNDS_5DEG);

            if(!measurements_within_bounds)
            {
                state = CONTROLLER_STANDBY;
            }

            break;
        }
    }

    return state;
}
