// Includes
#include "dac_init.h"

#include "driverlib.h"
#include "device.h"

void configureDAC(void)
{
    DAC_setReferenceVoltage(DACA_BASE, DAC_REF_ADC_VREFHI);
    DAC_setReferenceVoltage(DACB_BASE, DAC_REF_ADC_VREFHI);

    DAC_enableOutput(DACA_BASE);
    DAC_enableOutput(DACB_BASE);

    DAC_setShadowValue(DACA_BASE, 0);
    DAC_setShadowValue(DACB_BASE, 0);

    DEVICE_DELAY_US(10);
}
