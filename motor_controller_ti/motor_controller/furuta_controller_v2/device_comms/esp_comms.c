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

uint16_t angles_combined = 0U;

// Local functions
static void cs_deassert(void)
{
    GPIO_writePin(66, 0U);
    DEVICE_DELAY_US(10);
}

static void cs_assert(void)
{
    DEVICE_DELAY_US(10);
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

    angles_combined = raw_data_temp;

    const uint16_t angle1_temp = (raw_data_temp & 0xFFFFU);
    const uint16_t angle2_temp = (raw_data_temp >> 16U);

    if(angle1_temp <= 0x3FFFU)
    {
        *angle1 = angle1_temp * ESP_POSITION_SCALING;
    }

    if(angle2_temp <= 0x3FFFU)
    {
        *angle2 = angle2_temp * ESP_POSITION_SCALING;
    }
}
