/*
 * motor_control.c
 *
 *  Created on: 11-Dec-2022
 *      Author: Samvrit
 */

#include "motor_control.h"
#include "epwm_global.h"
#include "cpu_cla_shared.h"

#include "driverlib.h"
#include "device.h"

void motor_control_init(void)
{
    GPIO_writePin(122, 1U); // enable

    cla_inputs.enable = false;
    cla_inputs.override_direction = 0;
    cla_inputs.overrides_enable = false;
    cla_inputs.override_duty_percent = 0.0f;
    cla_inputs.torque_cmd = 0.0f;
}

void motor_control_motor_enable(void)
{
    GPIO_writePin(122, 1U);
}

void motor_control_motor_disable(void)
{
    GPIO_writePin(122, 0U);
}

void motor_control_fault_reset(void)
{
    GPIO_writePin(122, 1U);

    DEVICE_DELAY_US(25U);

    GPIO_writePin(122, 0U);

    DEVICE_DELAY_US(25U);

    GPIO_writePin(122, 1U);
}

float motor_control_get_current_fb(void)
{
    return cla_outputs.current_feedback;
}

float motor_control_get_v_bridge(void)
{
    return cla_outputs.v_bridge;
}

float motor_control_get_duty_ratio(void)
{
    return cla_outputs.duty;
}

float motor_control_get_torque_cmd(void)
{
    return cla_inputs.torque_cmd;
}

void motor_control_set_enable(const uint16_t enable)
{
    cla_inputs.enable = enable;
}

void motor_control_set_override_enable(const uint16_t enable)
{
    cla_inputs.overrides_enable = enable;
}

void motor_control_set_override_duty_ratio(const float duty)
{
    cla_inputs.override_duty_percent = duty;
}

void motor_control_set_override_direction(const uint16_t direction)
{
    cla_inputs.override_direction = direction;
}

void motor_control_set_torque_cmd(const float torque_cmd)
{
    cla_inputs.torque_cmd = torque_cmd;
}
