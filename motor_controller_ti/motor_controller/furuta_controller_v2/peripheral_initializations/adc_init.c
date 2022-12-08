// Includes
#include "adc_init.h"

#include "driverlib.h"
#include "device.h"

// Defines
#define ADC_PPB_REF_OFFSET (2048U)

//
// Function to configure and power up ADCA.
//
void initADC(void)
{
    ADC_disableConverter(ADCA_BASE);

    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_1_0);

    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
}

//
// Function to configure ADCA's SOC0 to be triggered by ePWM1.
//
void initADCSOC(void)
{
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN2, 15);

    ADC_setSOCPriority(ADCA_BASE, ADC_PRI_THRU_SOC5_HIPRI);

    ADC_setupPPB(ADCA_BASE, ADC_PPB_NUMBER1, ADC_SOC_NUMBER0);

    ADC_setPPBReferenceOffset(ADCA_BASE, ADC_PPB_NUMBER1, ADC_PPB_REF_OFFSET);

    ADC_enableConverter(ADCA_BASE);
    DEVICE_DELAY_US(1000);

    //
    // Set SOC0 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_enableContinuousMode(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
}
