#ifndef _CLA_ATAN_SHARED_H_
#define _CLA_ATAN_SHARED_H_

// Included Files
#include <stdint.h>
#include <stdbool.h>

#include "F2837xD_device.h"

// Defines


// Globals
struct cla_inputs_S
{
    float torque_cmd;
    bool enable;
    bool overrides_enable;
    float override_duty_percent;
    uint16_t override_direction;
};

struct cla_outputs_S
{
    float current_feedback;
    float v_bridge;
    float duty;
};

struct cpu_cla_shared_S
{
    float integrator;
};

//Task 1 (C) Variables
extern struct cla_inputs_S cla_inputs;
extern struct cla_outputs_S cla_outputs;
extern struct cpu_cla_shared_S cpu_cla_shared;

// Function Prototypes
__interrupt void motor_torque_control();
__interrupt void sw_task();

#endif //end of _CLA_ATAN_SHARED_H_ definition
