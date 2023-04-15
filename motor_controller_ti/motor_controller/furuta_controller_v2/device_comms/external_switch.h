/*
 * external_switch.h
 *
 *  Created on: 14-Apr-2023
 *      Author: Samvrit
 */

#ifndef DEVICE_COMMS_EXTERNAL_SWITCH_H_
#define DEVICE_COMMS_EXTERNAL_SWITCH_H_

#include <stdint.h>

extern uint16_t motor_enable_switch;

__interrupt void switch_interrupt(void);

#endif /* DEVICE_COMMS_EXTERNAL_SWITCH_H_ */
