/*
 * core_controls.h
 *
 *  Created on: 20-Nov-2022
 *      Author: Samvrit
 */

#ifndef CONTROLS_CORE_CONTROLS_H_
#define CONTROLS_CORE_CONTROLS_H_

#include <stdbool.h>

__interrupt void epwm3ISR(void);

void controller_init(void);

#endif /* CONTROLS_CORE_CONTROLS_H_ */
