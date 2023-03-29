/*
 * host_comms.c
 *
 *  Created on: 02-Jan-2023
 *      Author: Samvrit
 */

#include "host_comms.h"
#include "host_comms_shared.h"
#include "fast_logging.h"
#include "core_controls.h"
#include "motor_control.h"

#include "driverlib.h"

// Extern global variables

uint16_t host_rx_command_zero_position_offset = 0U;
uint16_t host_rx_command_motor_enable = 0U;

// Local types

union float_to_uint_U
{
    float value;
    uint16_t raw[2];
};

// Local variables
uint16_t host_rx_command_start_info_streaming = 0U;
uint16_t host_rx_command_trigger_fast_logging = 0U;
uint16_t host_rx_command_reset_motor = 0U;

fast_logging_states_E fast_logging_state;

// Helper functions
static inline void send_float(const float value, const uint16_t id)
{
    union float_to_uint_U data_to_send = { .value = value };

    SCI_writeCharNonBlocking(SCIA_BASE, id);   // identifier char

    for(uint16_t i = 0; i < 2; i++)
    {
        const uint16_t byte1 = __byte((int *)&data_to_send.raw[i], 0);
        const uint16_t byte2 = __byte((int *)&data_to_send.raw[i], 1);
        SCI_writeCharNonBlocking(SCIA_BASE, byte1);
        SCI_writeCharNonBlocking(SCIA_BASE, byte2);
    }
}

// ISR function
__interrupt void scibRXFIFOISR(void)
{
    uint16_t received_data[5] = { 0U };

    SCI_readCharArray(SCIA_BASE, (uint16_t *)&received_data, 5);

    switch(received_data[0])
    {
        case ZERO_POSITION_OFFSET:
            host_rx_command_zero_position_offset = 1U;
            break;

        case START_STREAMING_DATA:
            host_rx_command_start_info_streaming = 1U;
            break;

        case STOP_STREAMING_DATA:
            host_rx_command_start_info_streaming = 0U;
            break;

        case DUTY_RATIO_OVERRIDE:
        {
            union float_to_uint_U uint_to_float;

            uint_to_float.raw[0] = (received_data[2] << 8U) | received_data[1];
            uint_to_float.raw[1] = (received_data[4] << 8U) | received_data[3];

            motor_control_set_override_duty_ratio(uint_to_float.value);
            break;
        }

        case TORQUE_CMD_OVERRIDE:
        {
            union float_to_uint_U uint_to_float;

            uint_to_float.raw[0] = (received_data[2] << 8U) | received_data[1];
            uint_to_float.raw[1] = (received_data[4] << 8U) | received_data[3];

            motor_control_set_torque_cmd(uint_to_float.value);
            break;
        }

        case DIRECTION_TOGGLE:
        {
            motor_control_set_override_direction(received_data[1]);
            break;
        }

        case OVERRIDE_TOGGLE:
        {
            motor_control_set_override_enable(received_data[1]);
            break;
        }

        case MOTOR_ENABLE_TOGGLE:
        {
            host_rx_command_motor_enable = received_data[1];
            break;
        }

        case RESET_MOTOR:
        {
            host_rx_command_reset_motor = 1U;
            break;
        }

        case TRIGGER_FAST_LOGGING:
        {
            host_rx_command_trigger_fast_logging = 1U;
            break;
        }

        default:
            break;
    }

    SCI_resetRxFIFO(SCIA_BASE);

    SCI_clearOverflowStatus(SCIA_BASE);

    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

static void send_data_to_host(void)
{
    static int16_t slow_index = (int16_t)X_HAT_0;
    static int16_t fast_index = (int16_t)FAST_LOGGING_SIGNALS_READY;
    static uint16_t fast_logging_index = 0U;

    float x_hat[4] = { 0.0f };
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

    const bool fast_logging_inactive = (fast_logging_state != FAST_LOGGING_BUFFER_FULL);
    const bool fast_logging_ready = (fast_logging_state == FAST_LOGGING_BUFFER_FULL);

    float *fast_logging_buffer = fast_logging_get_buffer();

    const uint16_t fifo_empty_bins = SCI_FIFO_TX16 - SCI_getTxFIFOStatus(SCIA_BASE);

    if(host_rx_command_start_info_streaming && fast_logging_inactive)
    {
        switch(slow_index)
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
                    // send_float(x_hat[4], X_HAT_4);
                    send_float(0.0f, X_HAT_4);
                }
                break;
            }
            case X_HAT_5:
            {
                if(fifo_empty_bins >= 5U)
                {
                    // send_float(x_hat[5], X_HAT_5);
                    send_float(0.0f, X_HAT_5);
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
                    SCI_writeCharNonBlocking(SCIA_BASE, RLS_ERROR_BITFIELD);   // identifier char
                    SCI_writeCharNonBlocking(SCIA_BASE, rls_error_bitfield);
                }
                break;
            }
            case MOTOR_FAULT_FLAG:
            {
                if(fifo_empty_bins >= 2U)
                {
                    SCI_writeCharNonBlocking(SCIA_BASE, MOTOR_FAULT_FLAG);   // identifier char
                    SCI_writeCharNonBlocking(SCIA_BASE, motor_fault_flag);
                }
                break;
            }
            case CONTROLLER_STATE:
            {
                if(fifo_empty_bins >= 2U)
                {
                    SCI_writeCharNonBlocking(SCIA_BASE, CONTROLLER_STATE);   // identifier char
                    SCI_writeCharNonBlocking(SCIA_BASE, controller_state);
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

        if (++slow_index == (int16_t)SLOW_LOGGING_MAX)
        {
            slow_index = (int16_t)X_HAT_0;
        }
    }
    else if (fast_logging_ready)
    {
        switch(fast_index)
        {
            case FAST_LOGGING_SIGNALS_READY:
            {
                if(fifo_empty_bins >= 1U)
                {
                    SCI_writeCharNonBlocking(SCIA_BASE, FAST_LOGGING_SIGNALS_READY);   // identifier char
                }
                break;
            }
            case FAST_LOGGING_SIGNAL1:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(fast_logging_buffer[fast_logging_index], FAST_LOGGING_SIGNAL1);
                }
                break;
            }
            case FAST_LOGGING_SIGNAL2:
            {
                if(fifo_empty_bins >= 5U)
                {
                    send_float(fast_logging_buffer[FAST_LOGGING_BUFFER_SIZE + fast_logging_index], FAST_LOGGING_SIGNAL2);
                }

                if ((++fast_logging_index) >= FAST_LOGGING_BUFFER_SIZE)
                {
                    fast_index = (int16_t)FAST_LOGGING_SIGNALS_DONE;
                }

                break;
            }
            case FAST_LOGGING_SIGNALS_DONE:
            {
                if(fifo_empty_bins >= 1U)
                {
                    SCI_writeCharNonBlocking(SCIA_BASE, FAST_LOGGING_SIGNALS_DONE);   // identifier char
                }

                fast_logging_index = 0U;

                fast_index = FAST_LOGGING_SIGNALS_READY;

                fast_logging_clear_buffer();

                break;
            }
            default:
                break;
        }
        if(fast_index == FAST_LOGGING_SIGNALS_DONE)
        {
            // Don't increment fast_index
        }
        else if (++fast_index == (int16_t)FAST_LOGGING_MAX)
        {
            fast_index = (int16_t)FAST_LOGGING_SIGNAL1;
        }
    }
    else
    {
        SCI_resetTxFIFO(SCIA_BASE);
        slow_index = (int16_t)X_HAT_0;
        fast_index = (int16_t)FAST_LOGGING_SIGNALS_READY;
    }
}

// Public functions
void host_comms_200Hz_task(void)
{
    send_data_to_host();

    if (host_rx_command_reset_motor)
    {
        host_rx_command_reset_motor = 0U;
        motor_control_fault_reset();
    }
}

void host_comms_1kHz_task(void)
{
    const float torque_cmd = motor_control_get_torque_cmd();
    const float current_fb = motor_control_get_current_fb();

    static float torque_cmd_prev = 0.0f;

    const float signals[2] = { torque_cmd, current_fb };

    const bool trigger = host_rx_command_trigger_fast_logging && (torque_cmd > 0.0f) && (torque_cmd_prev <= 0.0f);

    torque_cmd_prev = torque_cmd;

    if(trigger)
    {
        host_rx_command_trigger_fast_logging = 0U;
    }

    fast_logging_state = fast_logging_step(trigger, signals);
}


