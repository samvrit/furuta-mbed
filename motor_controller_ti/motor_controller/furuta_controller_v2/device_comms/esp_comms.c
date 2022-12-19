/*
 * esp_comms.c
 *
 *  Created on: 18-Dec-2022
 *      Author: Samvrit
 */

#include "esp_comms.h"

#include "driverlib.h"
#include "device.h"

// Defines
#define SPI_N_WORDS (4U)

// Local types
union uint32_to_float_T
{
    float value;
    uint32_t raw;
};

// Local variables
union uint32_to_float_T esp_data;

// Local functions
static void cs_deassert(void)
{
    GPIO_writePin(66, 0U);
    DEVICE_DELAY_US(1);
}

static void cs_assert(void)
{
    DEVICE_DELAY_US(1);
    GPIO_writePin(66, 1U);
}

// Global functions
float esp_get_data(void)
{
    const uint16_t sData[SPI_N_WORDS] = {0x0U, 0x0U, 0x0U, 0x0U};

    cs_deassert();

    uint32_t raw_data_temp = 0U;

    for(int i = 0; i < SPI_N_WORDS; i++)
    {
        SPI_writeDataBlockingNonFIFO(SPIB_BASE, sData[i]);
        const uint16_t receive_data = SPI_readDataBlockingNonFIFO(SPIB_BASE);

        const uint16_t shift_count = 8U * ((SPI_N_WORDS - 1U) - i);

        raw_data_temp |= ( ((uint32_t)receive_data & 0xFFU) << shift_count );
    }

    cs_assert();

    esp_data.raw = raw_data_temp;

    return esp_data.value;
}
