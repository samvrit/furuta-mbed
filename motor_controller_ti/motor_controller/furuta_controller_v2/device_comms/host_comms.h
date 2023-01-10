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

__interrupt void scibRXFIFOISR(void);

void send_data_to_host(const float x_hat[6], const float measurements[6], const float torque_cmd, const uint16_t controller_state, const uint16_t rls_error_bitfield, const uint16_t motor_fault_flag);

#endif /* DEVICE_COMMS_HOST_COMMS_H_ */
