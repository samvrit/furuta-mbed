/*
 * dac_driver.c
 *
 *  Created on: 18-Feb-2023
 *      Author: Samvrit
 */

#include "dac_driver.h"

#include "driverlib.h"

#define DAC_OFFSET (1.5f)
#define VREF (3.0f)
#define MAX_DAC_VAL (4095U)

void dac_driver_set_value(dac_base_E dac_base, const float value, const float max_value)
{
    const float percent = value / fabsf(max_value);

    const float voltage_value = DAC_OFFSET + (percent * (VREF - DAC_OFFSET));

    const float dac_value = (voltage_value / VREF) * MAX_DAC_VAL;

    const uint32_t base = (dac_base == DAC_BASE_A) ? DACA_BASE : DACB_BASE;

    DAC_setShadowValue(base, (uint16_t)dac_value);
}
