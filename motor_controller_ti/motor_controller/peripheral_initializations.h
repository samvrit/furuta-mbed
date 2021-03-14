/*
 * initializations.h
 *
 *  Created on: Apr 8, 2020
 *      Author: Samvrit Srinivas
 */

#ifndef PERIPHERAL_INITIALIZATIONS_H_
#define PERIPHERAL_INITIALIZATIONS_H_

/*==================INCLUDES==================*/
#include <stdint.h>

/*==================FUNCTION PROTOTYPES==================*/
void configGPIOS(void);
void CLA_runTask(void);
void CLA_configClaMemory(void);
void CLA_initCpu1Cla(void);
void initADC(void);
void initEPWM(void);
void initADCSOC(void);
void initSCIBFIFO(void);
void configCPUTimer(uint32_t, uint32_t, uint16_t);
void initSPIA(void);
void initSPIB(void);



#endif /* PERIPHERAL_INITIALIZATIONS_H_ */
