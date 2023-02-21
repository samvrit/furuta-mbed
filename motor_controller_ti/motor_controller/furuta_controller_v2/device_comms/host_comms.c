/*
 * host_comms.c
 *
 *  Created on: 02-Jan-2023
 *      Author: Samvrit
 */

#include "host_comms.h"
#include "core_controls.h"
#include "motor_control.h"

#include "driverlib.h"

// Extern global variables

uint16_t host_rx_command_zero_position_offset = 0U;

// Local types

enum host_data_E
{
    X_HAT_0,
    X_HAT_1,
    X_HAT_2,
    X_HAT_3,
    X_HAT_4,
    X_HAT_5,

    TORQUE_CMD,

    MEASUREMENTS_0,
    MEASUREMENTS_1,
    MEASUREMENTS_2,

    RLS_ERROR_BITFIELD,
    MOTOR_FAULT_FLAG,

    CONTROLLER_STATE,

    CURRENT_FB,
    V_BRIDGE,
    DUTY_RATIO,

    DATA_MAX,
};

union float_to_uint_U
{
    float value;
    uint16_t raw[2];
};

// Local variables
uint16_t host_rx_command_start_info_streaming = 0U;

// Helper functions
static inline void send_float(const float value, const uint16_t id)
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

static void send_data_to_host(void)
{
    static int16_t index = (int16_t)X_HAT_0;

    float x_hat[6] = { 0.0f };
    float measurements[3] = { 0.0f };

    controller_get_measurements(measurements);
    controller_get_state_estimate(x_hat);
    const uint16_t rls_error_bitfield = controller_get_rls_error_bitfield();
    const float torque_cmd = controller_get_torque_cmd();
    const uint16_t controller_state = controller_get_controller_state();
    const uint16_t motor_fault_flag = controller_get_motor_fault();
    const float current_fb = motor_control_get_current_fb();
    const float v_bridge = motor_control_get_v_bridge();
    const float duty_ratio = motor_control_get_duty_ratio();

    const uint16_t fifo_empty_bins = SCI_FIFO_TX16 - SCI_getTxFIFOStatus(SCIB_BASE);

    if(host_rx_command_start_info_streaming)
    {
        switch(index)
        {
            case X_HAT_0:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(x_hat[0], X_HAT_0);
                }
                break;
            }
            case X_HAT_1:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(x_hat[1], X_HAT_1);
                }
                break;
            }
            case X_HAT_2:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(x_hat[2], X_HAT_2);
                }
                break;
            }
            case X_HAT_3:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(x_hat[3], X_HAT_3);
                }
                break;
            }
            case X_HAT_4:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(x_hat[4], X_HAT_4);
                }
                break;
            }
            case X_HAT_5:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(x_hat[5], X_HAT_5);
                }
                break;
            }
            case TORQUE_CMD:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(torque_cmd, TORQUE_CMD);
                }
                break;
            }
            case MEASUREMENTS_0:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(measurements[0], MEASUREMENTS_0);
                }
                break;
            }
            case MEASUREMENTS_1:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(measurements[1], MEASUREMENTS_1);
                }
                break;
            }
            case MEASUREMENTS_2:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(measurements[2], MEASUREMENTS_2);
                }
                break;
            }
            case RLS_ERROR_BITFIELD:
            {
                if(fifo_empty_bins >= 2U)
                {
                    SCI_writeCharNonBlocking(SCIB_BASE, RLS_ERROR_BITFIELD);   // identifier char
                    SCI_writeCharNonBlocking(SCIB_BASE, rls_error_bitfield);
                }
                break;
            }
            case MOTOR_FAULT_FLAG:
            {
                if(fifo_empty_bins >= 2U)
                {
                    SCI_writeCharNonBlocking(SCIB_BASE, MOTOR_FAULT_FLAG);   // identifier char
                    SCI_writeCharNonBlocking(SCIB_BASE, motor_fault_flag);
                }
                break;
            }
            case CONTROLLER_STATE:
            {
                if(fifo_empty_bins >= 2U)
                {
                    SCI_writeCharNonBlocking(SCIB_BASE, CONTROLLER_STATE);   // identifier char
                    SCI_writeCharNonBlocking(SCIB_BASE, controller_state);
                }
                break;
            }
            case CURRENT_FB:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(current_fb, CURRENT_FB);
                }
                break;
            }
            case V_BRIDGE:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(v_bridge, V_BRIDGE);
                }
                break;
            }
            case DUTY_RATIO:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(duty_ratio, DUTY_RATIO);
                }
                break;
            }
            default:
                break;
        }

        if (++index == (int16_t)DATA_MAX)
        {
            index = (int16_t)X_HAT_0;
        }
    }
    else
    {
        SCI_resetTxFIFO(SCIB_BASE);
        index = (int16_t)X_HAT_0;
    }
}

// Public functions
void host_comms_100Hz_task(void)
{
    send_data_to_host();
}


