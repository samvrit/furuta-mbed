// Included Files
#include "cpu_cla_shared.h"
#include "cla_init.h"
#include "adc_init.h"
#include "epwm_init.h"
#include "gpio_init.h"
#include "dac_init.h"
#include "spi_init.h"

#include "rls_comms.h"
#include "core_controls.h"

#include "driverlib.h"
#include "device.h"

// Defines

// Globals

#pragma DATA_SECTION(cla_inputs,"CpuToCla1MsgRAM");
struct cla_inputs_S cla_inputs = {.enable = false, .torque_cmd = 0.0f, .pwm_override_enable = false, .pwm_override_cmpa = 0U};

#pragma DATA_SECTION(cla_outputs,"Cla1ToCpuMsgRAM");
struct cla_outputs_S cla_outputs;

#pragma DATA_SECTION(cpu_cla_shared,"CLADataLS0")
struct cpu_cla_shared_S cpu_cla_shared = {.integrator = 0.0f};

uint16_t dac_value = 4095;
uint32_t rls_position = 0U;


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

    controller_init();

    initGPIO();

    initADC();

    initEPWM();

    initADCSOC();

    initSPIA();

    cla_configClaMemory();

    cla_initCpu1Cla1();

    configureDAC();

    // Enable Global Interrupts (INTM) and realtime interrupt (DGBM)

    EINT;
    ERTM;

    for(;;)
    {
        DAC_setShadowValue(DACA_BASE, dac_value);

        rls_position = rls_get_position();

        DEVICE_DELAY_US(10000U);
    }
}
