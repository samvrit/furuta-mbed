// Included Files
#include "cpu_cla_shared.h"
#include "cla_init.h"
#include "adc_init.h"
#include "epwm_init.h"
#include "gpio_init.h"

#include "driverlib.h"
#include "device.h"

// Defines

// Globals


#pragma DATA_SECTION(cla_inputs,"CpuToCla1MsgRAM");
struct cla_inputs_S cla_inputs;

#pragma DATA_SECTION(cla_outputs,"Cla1ToCpuMsgRAM");
struct cla_outputs_S cla_outputs;

#pragma DATA_SECTION(cpu_cla_shared,"CLADataLS0")
struct cpu_cla_shared_S cpu_cla_shared;


void main(void)
{
    // Initialize device clock and peripherals
    Device_init();

    // Disable pin locks and enable internal pullups.
    Device_initGPIO();

    // Initialize PIE and clear PIE registers. Disables CPU interrupts
    Interrupt_initModule();

    // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).
    Interrupt_initVectorTable();

    initGPIO();

    // Init ADC and SOC
    initADC();
    initEPWM();
    initADCSOC();

    // Configure the CLA memory spaces first followed by the CLA task vectors
    cla_configClaMemory();
    cla_initCpu1Cla1();

    CLA_forceTasks(CLA1_BASE,CLA_TASKFLAG_8);

    // Enable Global Interrupts (INTM) and realtime interrupt (DGBM)

    EINT;
    ERTM;

    for(;;)
    {
    }
}