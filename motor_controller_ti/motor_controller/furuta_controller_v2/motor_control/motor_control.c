/*
 * motor_control.c
 *
 *  Created on: 11-Dec-2022
 *      Author: Samvrit
 */

#include "motor_control.h"
#include "epwm_global.h"
#include "cpu_cla_shared.h"

void motor_control_init(void)
{
    cla_inputs.enable = false;
    cla_inputs.override_direction = 0;
    cla_inputs.overrides_enable = false;
    cla_inputs.override_duty_percent = 0.0f;
    cla_inputs.torque_cmd = 0.0f;
}
