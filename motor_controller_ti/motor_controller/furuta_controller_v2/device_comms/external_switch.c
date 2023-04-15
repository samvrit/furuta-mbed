/*
 * external_switch.c
 *
 *  Created on: 14-Apr-2023
 *      Author: Samvrit
 */

#include "external_switch.h"

#include "driverlib.h"

uint16_t motor_enable_switch = 0;

__interrupt void switch_interrupt(void)
{
    motor_enable_switch = !motor_enable_switch;

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
