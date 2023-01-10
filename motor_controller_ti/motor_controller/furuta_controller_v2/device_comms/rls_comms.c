/*
 * rls_comms.c
 *
 *  Created on: 10-Dec-2022
 *      Author: Samvrit
 */

#include "rls_comms.h"

#include "driverlib.h"
#include "device.h"

// Defines
#define RLS_POSITION_SCALING  (3.835186051e-4f)  // [rad/count] equal to (2*pi)/(2^14-1)

#define SPI_N_WORDS (4U)

// Local functions
static inline void cs_deassert(void)
{
    GPIO_writePin(61, 0U);
    DEVICE_DELAY_US(1);
}

static inline void cs_assert(void)
{
    DEVICE_DELAY_US(1);
    GPIO_writePin(61, 1U);
}

// Global functions
float rls_get_position(uint16_t* error_bitfield)
{
    const uint16_t sData[SPI_N_WORDS] = {0x6400U, 0x0U, 0x0U, 0x0U};

    cs_deassert();

    uint32_t raw_data_temp = 0U;

    for(int i = 0; i < SPI_N_WORDS; i++)
    {
        SPI_writeDataBlockingNonFIFO(SPIA_BASE, sData[i]);
        const uint16_t receive_data = SPI_readDataBlockingNonFIFO(SPIA_BASE);

        // Received data is big-endian. Convert it to little-endian
        const uint16_t shift_count = 8U * ((SPI_N_WORDS - 1U) - i);

        raw_data_temp |= ( ((uint32_t)receive_data & 0xFFU) << shift_count );
    }

    cs_assert();

    const uint16_t position_raw = (raw_data_temp >> 18U);

    const float position = position_raw * RLS_POSITION_SCALING;

    *error_bitfield = ((raw_data_temp & 0xFF00U) >> 8U);

    return position;
}
