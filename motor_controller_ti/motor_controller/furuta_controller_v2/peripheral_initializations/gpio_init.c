// Includes
#include "gpio_init.h"

#include "driverlib.h"


void initGPIO(void)
{
    // Motor PWM pin
    GPIO_setPinConfig(GPIO_0_EPWM1A);

    // Motor direction pin
    GPIO_setPinConfig(GPIO_1_GPIO1);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(1, GPIO_CORE_CPU1_CLA1);
}

