/*
 * host_comms.c
 *
 *  Created on: 02-Jan-2023
 *      Author: Samvrit
 */

#include "host_comms.h"

#include "driverlib.h"

// Extern global variables

uint16_t host_rx_command_zero_position_offset = 0U;

// Local types

union float_to_uint_U
{
    float value;
    uint16_t raw[2];
};

// Local variables
uint16_t host_rx_command_start_info_streaming = 0U;

// Helper functions
static inline void send_float(const float value, const char id)
{
    union float_to_uint_U data_to_send = { .value = value };

    SCI_writeCharNonBlocking(SCIB_BASE, id);   // identifier char

    for(uint16_t i = 0; i < 2; i++)
    {
        const uint16_t byte1 = __byte((int *)&data_to_send.raw[i], 0);
        const uint16_t byte2 = __byte((int *)&data_to_send.raw[i], 1);
        SCI_writeCharNonBlocking(SCIB_BASE, byte1);
        SCI_writeCharNonBlocking(SCIB_BASE, byte2);
    }
}

// ISR function
__interrupt void scibRXFIFOISR(void)
{
    uint16_t received_data = 0U;

    SCI_readCharArray(SCIB_BASE, &received_data, 1);

    switch(received_data)
    {
        case 'c':
            host_rx_command_zero_position_offset = 1U;
            break;

        case 's':
            host_rx_command_start_info_streaming = 1U;
            break;

        case 't':
            host_rx_command_start_info_streaming = 0U;
            break;

        default:
            break;
    }

    SCI_resetRxFIFO(SCIB_BASE);

    SCI_clearOverflowStatus(SCIB_BASE);

    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_RXFF);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

// Public functions
void send_data_to_host(const float x_hat[6], const float measurements[6], const float torque_cmd, const uint16_t controller_state, const uint16_t rls_error_bitfield, const uint16_t motor_fault_flag)
{
    static uint16_t index = 0;

    const uint16_t fifo_empty_bins = SCI_FIFO_TX16 - SCI_getTxFIFOStatus(SCIB_BASE);

    if(host_rx_command_start_info_streaming)
    {
        switch(index)
        {
            case 0:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(x_hat[0], 'a');
                }
                break;
            }
            case 1:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(x_hat[1], 'b');
                }
                break;
            }
            case 2:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(x_hat[2], 'c');
                }
                break;
            }
            case 3:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(x_hat[3], 'd');
                }
                break;
            }
            case 4:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(x_hat[4], 'e');
                }
                break;
            }
            case 5:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(x_hat[5], 'f');
                }
                break;
            }
            case 6:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(torque_cmd, 'g');
                }
                break;
            }
            case 7:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(measurements[0], 'h');
                }
                break;
            }
            case 8:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(measurements[1], 'i');
                }
                break;
            }
            case 9:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(measurements[2], 'j');
                }
                break;
            }
            case 10:
            {
                if(fifo_empty_bins >= 2U)
                {
                    SCI_writeCharNonBlocking(SCIB_BASE, 'k');   // identifier char
                    SCI_writeCharNonBlocking(SCIB_BASE, rls_error_bitfield);
                }
                break;
            }
            case 11:
            {
                if(fifo_empty_bins >= 2U)
                {
                    SCI_writeCharNonBlocking(SCIB_BASE, 'l');   // identifier char
                    SCI_writeCharNonBlocking(SCIB_BASE, motor_fault_flag);
                }
                break;
            }
            case 12:
            {
                if(fifo_empty_bins >= 2U)
                {
                    SCI_writeCharNonBlocking(SCIB_BASE, 'm');   // identifier char
                    SCI_writeCharNonBlocking(SCIB_BASE, controller_state);
                }
                break;
            }
            default:
                break;
        }

        if (++index == 13U)
        {
            index = 0U;
        }
    }
    else
    {
        SCI_resetTxFIFO(SCIB_BASE);
        index = 0U;
    }
}


