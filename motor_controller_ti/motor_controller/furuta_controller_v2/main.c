// Included Files
#include "cpu_cla_shared.h"
#include "cla_init.h"
#include "adc_init.h"
#include "epwm_init.h"
#include "gpio_init.h"
#include "dac_init.h"
#include "spi_init.h"
#include "sci_init.h"

#include "motor_control.h"
#include "dac_driver.h"

#include "core_controls.h"
#include "host_comms.h"
#include "external_switch.h"

#include "driverlib.h"
#include "device.h"

// Defines

// Globals

#pragma DATA_SECTION(cla_inputs,"CpuToCla1MsgRAM");
struct cla_inputs_S cla_inputs;

#pragma DATA_SECTION(cla_outputs,"Cla1ToCpuMsgRAM");
struct cla_outputs_S cla_outputs;

#pragma DATA_SECTION(cpu_cla_shared,"CLADataLS0")
struct cpu_cla_shared_S cpu_cla_shared = {.integrator = 0.0f};

uint16_t dac_value = 4095;

uint16_t motor_fault = 0;


void main(void)
{
    // Initialize device clock and peripheral
    Device_init();

    // Disable pin locks and enable internal pullups.
    Device_initGPIO();

    // Initialize PIE and clear PIE registers. Disables CPU interrupts
    Interrupt_initModule();

    // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).
    Interrupt_initVectorTable();

    initGPIO();

    controller_init();

    initADC();

    initEPWM();

    initADCSOC();

    initSPIA();

    initSPIB();

    cla_configClaMemory();

    cla_initCpu1Cla1();

    configureDAC();

    initSCI();

    // Enable Global Interrupts (INTM) and realtime interrupt (DGBM)

    EINT;
    ERTM;

    motor_control_init();

    for(;;)
    {
        if(task_200Hz_flag)
        {
            task_200Hz_flag = 0U;
            host_comms_200Hz_task();
        }

        if(task_1kHz_flag)
        {
            task_1kHz_flag = 0U;
            host_comms_1kHz_task();

            GPIO_writePin(41, (host_rx_command_motor_enable || motor_enable_switch));
        }
    }
}
