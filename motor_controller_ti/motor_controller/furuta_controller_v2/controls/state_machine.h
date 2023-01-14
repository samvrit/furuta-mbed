/*
 * state_machine.h
 *
 *  Created on: 10-Jan-2023
 *      Author: Samvrit
 */

#ifndef CONTROLS_STATE_MACHINE_H_
#define CONTROLS_STATE_MACHINE_H_

#include <stdint.h>

typedef enum
{
    CONTROLLER_INIT = 0U,
    CONTROLLER_STANDBY = 1U,
    CONTROLLER_QUALIFYING = 2U,
    CONTROLLER_ACTIVE = 3U
} controller_state_E;

controller_state_E state_machine_step(const float measurements[3]);

#endif /* CONTROLS_STATE_MACHINE_H_ */
