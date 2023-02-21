/*
 * dac_driver.h
 *
 *  Created on: 18-Feb-2023
 *      Author: Samvrit
 */

#ifndef DRIVERS_DAC_DRIVER_H_
#define DRIVERS_DAC_DRIVER_H_

#define DAC_OFFSET (1.5f)
#define VREF (3.0f)
#define MAX_DAC_VAL (4095U)

typedef enum
{
    DAC_BASE_A,
    DAC_BASE_B
} dac_base_E;

#if (!__TMS320C28XX_CLA__)

#include "driverlib.h"

static inline void dac_driver_set_value(dac_base_E dac_base, const float value, const float max_value)
{
    const float percent = value / fabsf(max_value);

    const float voltage_value = DAC_OFFSET + (percent * (VREF - DAC_OFFSET));

    const float dac_value = (voltage_value / VREF) * MAX_DAC_VAL;

    const uint32_t base = (dac_base == DAC_BASE_A) ? DACA_BASE : DACB_BASE;

    DAC_setShadowValue(base, (uint16_t)dac_value);
}

#else // (__TMS320C28XX_CLA__)

#include "F2837xD_device.h"

static inline void dac_driver_set_value(dac_base_E dac_base, const float value, const float max_value)
{
    const float percent = value / fabsf(max_value);

    const float voltage_value = DAC_OFFSET + (percent * (VREF - DAC_OFFSET));

    const float dac_value = (voltage_value / VREF) * MAX_DAC_VAL;

    switch (dac_base)
    {
        case DAC_BASE_A:
            DacaRegs.DACVALS.all = (uint16_t)dac_value;
            break;
        case DAC_BASE_B:
            DacbRegs.DACVALS.all = (uint16_t)dac_value;
            break;
        default:
            break;
    }
}

#endif // (!__TMS320C28XX_CLA__)

#endif /* DRIVERS_DAC_DRIVER_H_ */
