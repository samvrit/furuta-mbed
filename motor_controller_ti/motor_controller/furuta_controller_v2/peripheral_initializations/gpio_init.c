// Includes
#include "gpio_init.h"

#include "driverlib.h"


void initGPIO(void)
{
    // Motor PWM pin
    GPIO_setPinConfig(GPIO_0_EPWM1A);

    GPIO_setPinConfig(GPIO_4_EPWM3A);

    // Motor direction pin
    GPIO_setPinConfig(GPIO_1_GPIO1);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(1, GPIO_CORE_CPU1_CLA1);

    // Debug
    GPIO_setPinConfig(GPIO_2_GPIO2);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);

    // nFAULT (motor)
    GPIO_setPinConfig(GPIO_123_GPIO123);
    GPIO_setPadConfig(123, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(123, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(123, GPIO_QUAL_ASYNC);

    // SPI-A (RLS)
    GPIO_setPinConfig(GPIO_58_SPISIMOA);
    GPIO_setPadConfig(58, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(58, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(58, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_59_SPISOMIA);
    GPIO_setDirectionMode(59, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(59, GPIO_QUAL_ASYNC);
    GPIO_setMasterCore(59, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_60_SPICLKA);
    GPIO_setPadConfig(60, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(60, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(60, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_61_GPIO61);
    GPIO_setPadConfig(61, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(61, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(60, GPIO_CORE_CPU1);
}

