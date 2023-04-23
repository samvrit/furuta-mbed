/*
 * rls_comms.c
 *
 *  Created on: 10-Dec-2022
 *      Author: Samvrit
 */

#include "rls_comms.h"
#include "host_comms.h"

#include "driverlib.h"
#include "device.h"

// Defines
#define RLS_POSITION_SCALING  (3.83495197e-4f)  // [rad/count] equal to (2*pi)/(2^14)

#define RLS_POSITION_RAW_LPF_A (1.9996e-4f) // 5s time constant at 1kHz

#define SPI_N_WORDS (4U)

#define MAX(a,b) ((a) > (b) ? (a) : (b))

uint16_t position_offset = 0U;

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

// wraps a 14-bit unsigned integer from 0-16384 to an integer with the range -8192 to +8192
static inline int16_t wrap_count(const uint16_t count)
{
    const int16_t output = (count > 8192) ? ((int32_t)count - 16384) : count;

    return output;
}

// Global functions
#pragma CODE_SECTION(rls_get_position, ".TI.ramfunc")
void rls_get_position(float* position, float* velocity, uint16_t* error_bitfield, const float timestep)
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

    static float position_raw_lpf = 0.0f;

    if(host_rx_command_zero_position_offset)
    {
        host_rx_command_zero_position_offset = 0U;

        position_raw_lpf = (float)position_raw;

        position_offset = position_raw;
    }
    else
    {
        position_raw_lpf += (position_raw - position_raw_lpf) * RLS_POSITION_RAW_LPF_A;

        position_offset = (uint16_t)position_raw_lpf;
    }

    const int16_t diff = position_raw - position_offset;

    const uint16_t position_after_correction = (diff < 0) ? (16384U + diff) : diff;

    const int16_t position_int = wrap_count(position_after_correction);

    const float position_local = position_int * RLS_POSITION_SCALING;

    static float position_previous = 0.0f;

    const float velocity_local = (position_local - position_previous) / MAX(timestep, 0.001f);

    position_previous = position_local;

    *position = position_local;
    *velocity = velocity_local;
    *error_bitfield = ((raw_data_temp & 0xFF00U) >> 8U);
}
