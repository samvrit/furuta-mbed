#ifndef MPS_COMMS_H
#define MPS_COMMS_H

#include <stdint.h>

#include "driver/spi_master.h"

// read the current angle
uint16_t mps_get_angle(spi_device_handle_t * handle);

// set the current position as the zero-offset
void mps_set_zero_position_to_current_position(spi_device_handle_t * handle);

// set counter clockwise as the direction in which the angle increases
void mps_set_ccw_as_incrementing(spi_device_handle_t * handle);

#endif // MPS_COMMS_H
