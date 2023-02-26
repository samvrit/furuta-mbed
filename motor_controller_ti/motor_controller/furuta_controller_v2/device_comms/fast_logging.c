/*
 * fast_logging.c
 *
 *  Created on: 25-Feb-2023
 *      Author: Samvrit
 */

#include <stdint.h>

#include "fast_logging.h"

static float buffer[FAST_LOGGING_NUM_SIGNALS][FAST_LOGGING_BUFFER_SIZE] = { { 0.0f } };

static fast_logging_states_E state = FAST_LOGGING_STANDBY;

fast_logging_states_E fast_logging_step(const bool enable, const bool trigger, const float signals[FAST_LOGGING_NUM_SIGNALS])
{
    static uint16_t index = 0U;

    switch (state)
    {
        case FAST_LOGGING_STANDBY:
        {
            index = 0U;
            if (enable || trigger)
            {
                state = FAST_LOGGING_LOGGING;
            }
            break;
        }
        case FAST_LOGGING_LOGGING:
        {
            if (!enable)
            {
                state = FAST_LOGGING_STANDBY;
                break;
            }

            for (uint16_t i = 0; i < FAST_LOGGING_NUM_SIGNALS; i++)
            {
                buffer[i][index] = signals[i];
            }

            if (++index >= FAST_LOGGING_BUFFER_SIZE)
            {
                state = FAST_LOGGING_BUFFER_FULL;
            }

            break;
        }
        case FAST_LOGGING_BUFFER_FULL:
        {
            index = 0U;
            break;
        }
        default:
            break;
    }

    return state;
}

float * fast_logging_get_buffer(void)
{
    return &buffer[0][0];
}

void fast_logging_clear_buffer(void)
{
    for (uint16_t i = 0; i < FAST_LOGGING_NUM_SIGNALS; i++)
    {
        for (uint16_t j = 0; j < FAST_LOGGING_BUFFER_SIZE; j++)
        {
            buffer[i][j] = 0.0f;
        }

    }

    state = FAST_LOGGING_STANDBY;
}
