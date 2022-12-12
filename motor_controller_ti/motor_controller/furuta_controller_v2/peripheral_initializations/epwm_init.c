// Includes
#include "epwm_init.h"
#include "epwm_global.h"
#include "core_controls.h"

#include "driverlib.h"

// Defines
#define EPWM1_TIMER_TBPRD  MOTOR_CONTROL_TBPRD
#define EPWM3_TIMER_TBPRD  (5000U)  // 10kHz with clock prescaler equal to 4

// Public Functions
void initEPWM(void)
{
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_A);

    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO_OR_PERIOD);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);

    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);

    SysCtl_setEPWMClockDivider(SYSCTL_EPWMCLK_DIV_1);

    // EPWM1 - Motor control

    EPWM_setClockPrescaler(EPWM1_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    EPWM_setTimeBasePeriod(EPWM1_BASE, 0);
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0);
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 0);
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_setCounterCompareShadowLoadMode(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);

    EPWM_setTimeBasePeriod(EPWM1_BASE, EPWM1_TIMER_TBPRD);

    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, EPWM1_TIMER_TBPRD/2);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);

    // EPWM3 - ISR

    Interrupt_register(INT_EPWM3, &epwm3ISR);

    EPWM_setClockPrescaler(EPWM3_BASE, EPWM_CLOCK_DIVIDER_4, EPWM_HSCLOCK_DIVIDER_1);

    EPWM_setTimeBasePeriod(EPWM3_BASE, 0);
    EPWM_setTimeBaseCounter(EPWM3_BASE, 0);
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, 0);
    EPWM_setTimeBaseCounterMode(EPWM3_BASE, EPWM_COUNTER_MODE_UP);
    EPWM_setCounterCompareShadowLoadMode(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);

    EPWM_setTimeBasePeriod(EPWM3_BASE, EPWM3_TIMER_TBPRD);

    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, EPWM3_TIMER_TBPRD/2);

    EPWM_setInterruptSource(EPWM3_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM3_BASE);
    EPWM_setInterruptEventCount(EPWM3_BASE, 1U);

    EPWM_clearEventTriggerInterruptFlag(EPWM3_BASE);

    Interrupt_enable(INT_EPWM3);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM3);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

