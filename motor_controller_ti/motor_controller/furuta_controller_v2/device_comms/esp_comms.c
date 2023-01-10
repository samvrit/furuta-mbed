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
#define SPI_N_WORDS (2U)

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
    const uint16_t sData[SPI_N_WORDS] = {0x4100U, 0x4200U};   // Dummy characters (AB)

    cs_deassert();

    SPI_writeDataBlockingNonFIFO(SPIB_BASE, sData[0]);
    const uint16_t angle1_raw = SPI_readDataBlockingNonFIFO(SPIB_BASE);

    SPI_writeDataBlockingNonFIFO(SPIB_BASE, sData[1]);
    const uint16_t angle2_raw = SPI_readDataBlockingNonFIFO(SPIB_BASE);

    cs_assert();

    *angle1 = angle1_raw * ESP_POSITION_SCALING;

    *angle2 = angle2_raw * ESP_POSITION_SCALING;
}
