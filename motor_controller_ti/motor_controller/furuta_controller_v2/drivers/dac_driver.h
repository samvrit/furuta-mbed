/*
 * dac_driver.h
 *
 *  Created on: 18-Feb-2023
 *      Author: Samvrit
 */

#ifndef DRIVERS_DAC_DRIVER_H_
#define DRIVERS_DAC_DRIVER_H_

typedef enum
{
    DAC_BASE_A,
    DAC_BASE_B
} dac_base_E;

void dac_driver_set_value(dac_base_E dac_base, const float value, const float max_value);

#endif /* DRIVERS_DAC_DRIVER_H_ */
