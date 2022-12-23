/*
 * motor_control.h
 *
 *  Created on: 11-Dec-2022
 *      Author: Samvrit
 */

#ifndef MOTOR_CONTROL_MOTOR_CONTROL_H_
#define MOTOR_CONTROL_MOTOR_CONTROL_H_

void motor_control_init(void);

void motor_control_motor_enable(void);
void motor_control_motor_disable(void);
void motor_control_fault_reset(void);

#endif /* MOTOR_CONTROL_MOTOR_CONTROL_H_ */
