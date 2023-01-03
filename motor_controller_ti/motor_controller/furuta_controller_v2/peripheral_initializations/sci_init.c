/*
 * sci_init.c
 *
 *  Created on: 30-Dec-2022
 *      Author: Samvrit
 */

#include "sci_init.h"
#include "host_comms.h"

#include "driverlib.h"
#include "device.h"

void initSCI(void)
{
    Interrupt_register(INT_SCIB_RX, scibRXFIFOISR);

    // 8 char bits, 1 stop bit, no parity. Baud rate is 115200.
    SCI_setConfig(SCIB_BASE, DEVICE_LSPCLK_FREQ, 115200, (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE));
    SCI_enableModule(SCIB_BASE);
    SCI_resetChannels(SCIB_BASE);
    SCI_enableFIFO(SCIB_BASE);

    // RX and FIFO Interrupts Enabled
    SCI_enableInterrupt(SCIB_BASE, SCI_INT_RXFF);
    SCI_disableInterrupt(SCIB_BASE, SCI_INT_RXERR);

    SCI_setFIFOInterruptLevel(SCIB_BASE, SCI_FIFO_TX16, SCI_FIFO_RX1);
    SCI_performSoftwareReset(SCIB_BASE);

    SCI_resetTxFIFO(SCIB_BASE);
    SCI_resetRxFIFO(SCIB_BASE);

    Interrupt_enable(INT_SCIB_RX);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

