/*
 * rls_comms.h
 *
 *  Created on: 10-Dec-2022
 *      Author: Samvrit
 */

#ifndef DEVICE_COMMS_RLS_COMMS_H_
#define DEVICE_COMMS_RLS_COMMS_H_

#include <stdint.h>

void rls_get_position(float* position, float* velocity, uint16_t* error_bitfield, const float timestep);

#endif /* DEVICE_COMMS_RLS_COMMS_H_ */
