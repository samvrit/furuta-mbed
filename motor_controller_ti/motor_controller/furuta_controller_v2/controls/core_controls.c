// Includes
#include "core_controls.h"
#include "observer_controller.h"

#include "driverlib.h"


// Private data

const float Ts = 1e-4f;
float x_hat[6] = {0.0f};
float torque_cmd = 0.0f;

// Public Functions


__interrupt void epwm3ISR(void)
{
    GPIO_writePin(2, 1);

    const float measurement[6] = { 0 };
    observer_step(measurement, true, x_hat);

    torque_cmd = control_output(x_hat, Ts);

    GPIO_writePin(2, 0);

    EPWM_clearEventTriggerInterruptFlag(EPWM3_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

void controller_init(void)
{
    observer_init(x_hat, Ts);

    for(int i = 0; i < 20000; i++)
    {
        covariance_matrix_step();
    }
}
