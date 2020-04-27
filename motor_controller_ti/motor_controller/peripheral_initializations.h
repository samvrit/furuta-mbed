/*
 * initializations.h
 *
 *  Created on: Apr 8, 2020
 *      Author: Samvrit Srinivas
 */

#ifndef PERIPHERAL_INITIALIZATIONS_H_
#define PERIPHERAL_INITIALIZATIONS_H_

/*==================INCLUDES==================*/
#include "driverlib.h"
#include "device.h"

/*==================FUNCTION PROTOTYPES==================*/
void CLA_runTask(void);
void CLA_configClaMemory(void);
void CLA_initCpu1Cla1(void);
void initADC(void);
void initEPWM(void);
void initADCSOC(void);
void initSCIBFIFO(void);
void initEQEP(void);
void initCPUTimer(void);
void configCPUTimer(uint32_t, uint16_t);



#endif /* PERIPHERAL_INITIALIZATIONS_H_ */
