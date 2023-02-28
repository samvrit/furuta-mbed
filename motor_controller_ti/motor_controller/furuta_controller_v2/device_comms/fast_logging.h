/*
 * fast_logging.h
 *
 *  Created on: 25-Feb-2023
 *      Author: Samvrit
 */

#ifndef DEVICE_COMMS_FAST_LOGGING_H_
#define DEVICE_COMMS_FAST_LOGGING_H_

#include <stdbool.h>

#define FAST_LOGGING_NUM_SIGNALS (2U)
#define FAST_LOGGING_BUFFER_SIZE (1000U)

typedef enum
{
    FAST_LOGGING_STANDBY,
    FAST_LOGGING_LOGGING,
    FAST_LOGGING_BUFFER_FULL,
} fast_logging_states_E;

fast_logging_states_E fast_logging_step(const bool enable, const bool trigger, const float signals[FAST_LOGGING_NUM_SIGNALS]);

float * fast_logging_get_buffer(void);

void fast_logging_clear_buffer(void);

#endif /* DEVICE_COMMS_FAST_LOGGING_H_ */
