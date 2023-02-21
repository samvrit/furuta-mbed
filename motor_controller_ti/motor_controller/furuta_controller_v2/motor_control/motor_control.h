/*
 * motor_control.h
 *
 *  Created on: 11-Dec-2022
 *      Author: Samvrit
 */

#ifndef MOTOR_CONTROL_MOTOR_CONTROL_H_
#define MOTOR_CONTROL_MOTOR_CONTROL_H_

#include <stdint.h>

void motor_control_init(void);

void motor_control_motor_enable(void);
void motor_control_motor_disable(void);
void motor_control_fault_reset(void);

float motor_control_get_current_fb(void);
float motor_control_get_v_bridge(void);
float motor_control_get_duty_ratio(void);

void motor_control_set_enable(const uint16_t enable);
void motor_control_set_override_enable(const uint16_t enable);
void motor_control_set_override_duty_ratio(const float duty);
void motor_control_set_override_direction(const uint16_t direction);
void motor_control_set_torque_cmd(const float torque_cmd);

#endif /* MOTOR_CONTROL_MOTOR_CONTROL_H_ */
