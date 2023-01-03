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
void send_data_to_host(const float x_hat[6], const float measurements[6], const uint16_t rls_error_bitfield, const uint16_t motor_fault_flag)
{
    static uint16_t index = 0;

    if(host_rx_command_start_info_streaming)
    {
        switch(index)
        {
            case 0:
            {
                union float_to_uint_U data_to_send = { .value = x_hat[0] };

                SCI_writeCharNonBlocking(SCIB_BASE, 'a');   // identifier char

                for(uint16_t i = 0; i < 2; i++)
                {
                    const uint16_t byte1 = __byte((int *)&data_to_send.raw[i], 0);
                    const uint16_t byte2 = __byte((int *)&data_to_send.raw[i], 1);
                    SCI_writeCharNonBlocking(SCIB_BASE, byte1);
                    SCI_writeCharNonBlocking(SCIB_BASE, byte2);
                }
                break;
            }
            case 1:
            {
                union float_to_uint_U data_to_send = { .value = x_hat[1] };

                SCI_writeCharNonBlocking(SCIB_BASE, 'b');   // identifier char

                for(uint16_t i = 0; i < 2; i++)
                {
                    const uint16_t byte1 = __byte((int *)&data_to_send.raw[i], 0);
                    const uint16_t byte2 = __byte((int *)&data_to_send.raw[i], 1);
                    SCI_writeCharNonBlocking(SCIB_BASE, byte1);
                    SCI_writeCharNonBlocking(SCIB_BASE, byte2);
                }
                break;
            }
            case 2:
            {
                union float_to_uint_U data_to_send = { .value = x_hat[2] };

                SCI_writeCharNonBlocking(SCIB_BASE, 'c');   // identifier char

                for(uint16_t i = 0; i < 2; i++)
                {
                    const uint16_t byte1 = __byte((int *)&data_to_send.raw[i], 0);
                    const uint16_t byte2 = __byte((int *)&data_to_send.raw[i], 1);
                    SCI_writeCharNonBlocking(SCIB_BASE, byte1);
                    SCI_writeCharNonBlocking(SCIB_BASE, byte2);
                }
                break;
            }
            case 3:
            {
                union float_to_uint_U data_to_send = { .value = x_hat[3] };

                SCI_writeCharNonBlocking(SCIB_BASE, 'd');   // identifier char

                for(uint16_t i = 0; i < 2; i++)
                {
                    const uint16_t byte1 = __byte((int *)&data_to_send.raw[i], 0);
                    const uint16_t byte2 = __byte((int *)&data_to_send.raw[i], 1);
                    SCI_writeCharNonBlocking(SCIB_BASE, byte1);
                    SCI_writeCharNonBlocking(SCIB_BASE, byte2);
                }
                break;
            }
            case 4:
            {
                union float_to_uint_U data_to_send = { .value = x_hat[4] };

                SCI_writeCharNonBlocking(SCIB_BASE, 'e');   // identifier char

                for(uint16_t i = 0; i < 2; i++)
                {
                    const uint16_t byte1 = __byte((int *)&data_to_send.raw[i], 0);
                    const uint16_t byte2 = __byte((int *)&data_to_send.raw[i], 1);
                    SCI_writeCharNonBlocking(SCIB_BASE, byte1);
                    SCI_writeCharNonBlocking(SCIB_BASE, byte2);
                }
                break;
            }
            case 5:
            {
                union float_to_uint_U data_to_send = { .value = x_hat[5] };

                SCI_writeCharNonBlocking(SCIB_BASE, 'f');   // identifier char

                for(uint16_t i = 0; i < 2; i++)
                {
                    const uint16_t byte1 = __byte((int *)&data_to_send.raw[i], 0);
                    const uint16_t byte2 = __byte((int *)&data_to_send.raw[i], 1);
                    SCI_writeCharNonBlocking(SCIB_BASE, byte1);
                    SCI_writeCharNonBlocking(SCIB_BASE, byte2);
                }
                break;
            }
            case 6:
            {
                union float_to_uint_U data_to_send = { .value = measurements[0] };

                SCI_writeCharNonBlocking(SCIB_BASE, 'g');   // identifier char

                for(uint16_t i = 0; i < 2; i++)
                {
                    const uint16_t byte1 = __byte((int *)&data_to_send.raw[i], 0);
                    const uint16_t byte2 = __byte((int *)&data_to_send.raw[i], 1);
                    SCI_writeCharNonBlocking(SCIB_BASE, byte1);
                    SCI_writeCharNonBlocking(SCIB_BASE, byte2);
                }
                break;
            }
            case 7:
            {
                union float_to_uint_U data_to_send = { .value = measurements[1] };

                SCI_writeCharNonBlocking(SCIB_BASE, 'h');   // identifier char

                for(uint16_t i = 0; i < 2; i++)
                {
                    const uint16_t byte1 = __byte((int *)&data_to_send.raw[i], 0);
                    const uint16_t byte2 = __byte((int *)&data_to_send.raw[i], 1);
                    SCI_writeCharNonBlocking(SCIB_BASE, byte1);
                    SCI_writeCharNonBlocking(SCIB_BASE, byte2);
                }
                break;
            }
            case 8:
            {
                union float_to_uint_U data_to_send = { .value = measurements[2] };

                SCI_writeCharNonBlocking(SCIB_BASE, 'i');   // identifier char

                for(uint16_t i = 0; i < 2; i++)
                {
                    const uint16_t byte1 = __byte((int *)&data_to_send.raw[i], 0);
                    const uint16_t byte2 = __byte((int *)&data_to_send.raw[i], 1);
                    SCI_writeCharNonBlocking(SCIB_BASE, byte1);
                    SCI_writeCharNonBlocking(SCIB_BASE, byte2);
                }
                break;
            }
            case 9:
            {
                SCI_writeCharNonBlocking(SCIB_BASE, 'j');   // identifier char
                SCI_writeCharNonBlocking(SCIB_BASE, rls_error_bitfield);
                break;
            }
            case 10:
            {
                SCI_writeCharNonBlocking(SCIB_BASE, 'k');   // identifier char
                SCI_writeCharNonBlocking(SCIB_BASE, motor_fault_flag);
                break;
            }
            default:
                break;
        }

        if (++index == 11U)
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


