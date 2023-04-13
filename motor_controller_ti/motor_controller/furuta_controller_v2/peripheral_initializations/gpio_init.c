// Includes
#include "gpio_init.h"

#include "driverlib.h"


void initGPIO(void)
{
    // Motor PWM pins
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setPinConfig(GPIO_1_EPWM1B);

    GPIO_setPinConfig(GPIO_4_EPWM3A);

    // Debug
    GPIO_setPinConfig(GPIO_2_GPIO2);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);

    // LED1
    GPIO_setPinConfig(GPIO_52_GPIO52);
    GPIO_setPadConfig(52, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(52, GPIO_DIR_MODE_OUT);

    // nFAULT (motor)
    GPIO_setPinConfig(GPIO_123_GPIO123);
    GPIO_setPadConfig(123, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(123, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(123, GPIO_QUAL_ASYNC);

    // nSLEEP (motor)
    GPIO_setPinConfig(GPIO_122_GPIO122);
    GPIO_setPadConfig(122, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(122, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(122, GPIO_QUAL_ASYNC);

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

    // SPI-B (ESP)
    GPIO_setPinConfig(GPIO_63_SPISIMOB);
    GPIO_setPadConfig(63, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(63, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(63, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_64_SPISOMIB);
    GPIO_setDirectionMode(64, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(64, GPIO_QUAL_ASYNC);
    GPIO_setMasterCore(64, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_65_SPICLKB);
    GPIO_setPadConfig(65, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(65, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(65, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_66_GPIO66);
    GPIO_setPadConfig(66, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(66, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(66, GPIO_CORE_CPU1);

    // SCI-A
    GPIO_setPinConfig(GPIO_43_SCIRXDA);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(43, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_42_SCITXDA);
    GPIO_setDirectionMode(42, GPIO_DIR_MODE_OUT);
}

