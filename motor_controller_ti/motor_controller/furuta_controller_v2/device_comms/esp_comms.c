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

#define ESP_POSITION_SCALING  (3.835186051e-4f)  // [rad/count] equal to (2*pi)/(2^14-1)

// Local functions
static void cs_deassert(void)
{
    GPIO_writePin(66, 0U);
}

static void cs_assert(void)
{
    GPIO_writePin(66, 1U);
}

// Global functions
void esp_get_data(float * angle1, float * angle2)
{
    const uint16_t sData[SPI_N_WORDS] = {0x4100U, 0x4200U, 0x4300U, 0x4400U};   // Dummy characters (ABCD)

    cs_deassert();

    uint32_t raw_data_temp = 0U;

    for(int i = 0; i < SPI_N_WORDS; i++)
    {
        SPI_writeDataBlockingNonFIFO(SPIB_BASE, sData[i]);
        const uint16_t receive_data = SPI_readDataBlockingNonFIFO(SPIB_BASE);

        const uint16_t shift_count = 8U * i;

        raw_data_temp |= ( ((uint32_t)receive_data & 0xFFU) << shift_count );
    }

    cs_assert();

    *angle1 = (raw_data_temp & 0xFFFFU) * ESP_POSITION_SCALING;

    *angle2 = (raw_data_temp >> 16U) * ESP_POSITION_SCALING;
}
