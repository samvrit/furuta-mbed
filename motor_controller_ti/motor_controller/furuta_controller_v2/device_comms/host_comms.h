/*
 * host_comms.h
 *
 *  Created on: 02-Jan-2023
 *      Author: Samvrit
 */

#ifndef DEVICE_COMMS_HOST_COMMS_H_
#define DEVICE_COMMS_HOST_COMMS_H_

#include <stdint.h>

extern uint16_t host_rx_command_zero_position_offset;
extern uint16_t host_rx_command_motor_enable;

__interrupt void scibRXFIFOISR(void);

void host_comms_200Hz_task(void);
void host_comms_1kHz_task(void);

#endif /* DEVICE_COMMS_HOST_COMMS_H_ */
