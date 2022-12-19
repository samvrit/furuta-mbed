/*
 * spi_init.c
 *
 *  Created on: 10-Dec-2022
 *      Author: Samvrit
 */

#include "spi_init.h"

#include "driverlib.h"
#include "device.h"

void initSPIA()
{
    SPI_disableModule(SPIA_BASE);

    // SPI configuration. Use a 500kHz SPICLK and 16-bit word size.
    SPI_setConfig(SPIA_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0, SPI_MODE_MASTER, 1000000, 8);
    SPI_disableLoopback(SPIA_BASE);
    SPI_setEmulationMode(SPIA_BASE, SPI_EMULATION_STOP_AFTER_TRANSMIT);

    SPI_disableFIFO(SPIA_BASE);

    // Configuration complete. Enable the module.
    SPI_enableModule(SPIA_BASE);
}

void initSPIB()
{
    SPI_disableModule(SPIB_BASE);

    // SPI configuration. Use a 500kHz SPICLK and 16-bit word size.
    SPI_setConfig(SPIB_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0, SPI_MODE_MASTER, 1000000, 8);
    SPI_disableLoopback(SPIB_BASE);
    SPI_setEmulationMode(SPIB_BASE, SPI_EMULATION_STOP_AFTER_TRANSMIT);

    SPI_disableFIFO(SPIB_BASE);

    // Configuration complete. Enable the module.
    SPI_enableModule(SPIB_BASE);
}
