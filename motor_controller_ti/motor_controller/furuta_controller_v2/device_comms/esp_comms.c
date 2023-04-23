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

#define ESP_POSITION_SCALING  (3.83495197e-4f)  // [rad/count] equal to (2*pi)/(2^14)

#define MAX(a,b) ((a) > (b) ? (a) : (b))

uint16_t angles_combined = 0U;

// Local functions
static inline void cs_deassert(void)
{
    GPIO_writePin(66, 0U);
    DEVICE_DELAY_US(5);
}

static inline void cs_assert(void)
{
    DEVICE_DELAY_US(5);
    GPIO_writePin(66, 1U);
}

// wraps a 14-bit unsigned integer from 0-16384 to an integer with the range -8192 to +8192
static inline int16_t wrap_count(const uint16_t count)
{
    const int16_t output = (count > 8192) ? ((int32_t)count - 16384) : count;

    return output;
}

// Global functions
#pragma CODE_SECTION(esp_get_data, ".TI.ramfunc")
void esp_get_data(float * angle1, float * angle2, float * velocity1, float * velocity2, const float timestep)
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

    static float angle1_prev = 0.0f;
    static float angle2_prev = 0.0f;

    if(angle1_temp <= 16383U)  // check for sane values (0 to 2^14-1)
    {
        const int16_t angle1_int = wrap_count(angle1_temp);
        const float angle1_local = angle1_int * ESP_POSITION_SCALING;
        const float velocity_local = (angle1_local - angle1_prev) / MAX(timestep, 0.001f);

        angle1_prev = angle1_local;

        *angle1 = angle1_local;
        *velocity1 = velocity_local;
    }

    if(angle2_temp <= 16383U)  // check for sane values (0 to 2^14-1)
    {
        const int16_t angle2_int = wrap_count((angle2_temp & 0x3FFFU));
        const float angle2_local = angle2_int * ESP_POSITION_SCALING;
        const float velocity_local = (angle2_local - angle2_prev) / MAX(timestep, 0.001f);

        angle2_prev = angle2_local;

        *angle2 = angle2_local;
        *velocity2 = velocity_local;
    }
}
