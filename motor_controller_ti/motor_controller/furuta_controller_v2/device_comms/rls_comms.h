/*
 * rls_comms.h
 *
 *  Created on: 10-Dec-2022
 *      Author: Samvrit
 */

#ifndef DEVICE_COMMS_RLS_COMMS_H_
#define DEVICE_COMMS_RLS_COMMS_H_

#include <stdint.h>

uint32_t rls_get_position(void);

__interrupt void spiRxFIFOISR(void);

#endif /* DEVICE_COMMS_RLS_COMMS_H_ */
