/*
 * rls_comms.c
 *
 *  Created on: 10-Dec-2022
 *      Author: Samvrit
 */

#include "rls_comms.h"

#include "driverlib.h"
#include "device.h"

uint32_t raw_data = 0U;
uint32_t position_data = 0U;


// Local functions

static void cs_deassert(void)
{
    GPIO_writePin(61, 0U);
    DEVICE_DELAY_US(1);
}

static void cs_assert(void)
{
    DEVICE_DELAY_US(1);
    GPIO_writePin(61, 1U);
}

// Global functions

uint32_t rls_get_position(void)
{
    const uint16_t sData[4] = {0x6400U, 0x0U, 0x0U, 0x0U};
    uint16_t rData[4] = {0};

    cs_deassert();

    uint32_t raw_data_temp = 0U;

    for(int i = 0; i < 4; i++)
    {
        SPI_writeDataBlockingNonFIFO(SPIA_BASE, sData[i]);
        rData[i] = SPI_readDataBlockingNonFIFO(SPIA_BASE);

        raw_data_temp |= ( ( ((uint32_t)rData[i]) & 0xFFU ) << (8U*(3U-i)) );
    }

    cs_assert();

    raw_data = raw_data_temp;

    position_data = (raw_data >> 20U);

    return raw_data;
}
