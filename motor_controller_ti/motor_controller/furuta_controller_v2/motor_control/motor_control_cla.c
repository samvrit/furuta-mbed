// Included Files
#include "cpu_cla_shared.h"
#include "epwm_global.h"


// Defines

#define CURRENT_SCALING (0.003052503053f)   // [A/adc_resolution] = (3/4095)*(5/3)*(1/0.4)
#define CURRENT_BIAS_LPF_A (2.5e-6f)    // 4s time constant at 100kHz
#define TORQUE_CONSTANT (0.968f)    // [Nm/A]

#define CURRENT_CONTROLLER_KP (46.576f)       // [V/A]
#define CURRENT_CONTROLLER_KI (23.52902968f)  // [V/A] this term is already multiplied by the timestep (1e-5)
#define CURRENT_CONTROLLER_I_TERM_MAX (20.0f) // [V/A]

#define V_BRIDGE_MAX (12.0f)    // [V] DC voltage

#define MAX(a, b) __mmaxf32((a), (b))
#define MIN(a, b) __mminf32((a), (b))

#define SAT(x, max, min) (MAX(MIN((x), (max)), (min)))

// Globals
float pi_control_i_term;
float current_bias;

// Private function declarations
static inline float get_current_feedback_from_adc(void);
static inline float accumulate_current_bias(const float current_val_raw, const bool enable);
static inline float pi_control(const float error);
static inline float calculate_duty_percent(const float v_bridge);
static inline void set_counter_compare_and_direction_pin(const float duty_percent, const bool enable, const float override_duty_percent, const uint16_t direction_override, const bool override_enable);

// CLA task
__interrupt void motor_torque_control(void)
{
    const float current_val_raw = get_current_feedback_from_adc();

    // Accumulate bias when current controller is not active
    const float current_feedback = accumulate_current_bias(current_val_raw, !cla_inputs.enable);

    const float current_ref = cla_inputs.torque_cmd / TORQUE_CONSTANT;

    const float error = current_ref - current_feedback;

    const float v_bridge = pi_control(error);

    const float duty_percent = calculate_duty_percent(v_bridge);

    set_counter_compare_and_direction_pin(duty_percent, cla_inputs.enable, cla_inputs.override_duty_percent, cla_inputs.override_direction, cla_inputs.overrides_enable);

    // Set outputs
    cla_outputs.v_bridge = v_bridge;
    cla_outputs.duty = duty_percent;
    cla_outputs.current_feedback = current_feedback;
}

// Private functions
static inline float get_current_feedback_from_adc(void)
{
    const int32_t adc_raw = AdcaResultRegs.ADCPPB1RESULT.all;   // post processing block already has the offset subtracted
    const float current_val_raw = adc_raw * CURRENT_SCALING;

    return current_val_raw;
}

static inline float accumulate_current_bias(const float current_val_raw, const bool enable)
{
    if(enable)
    {
        current_bias += (current_val_raw - current_bias) * CURRENT_BIAS_LPF_A;
    }

    const float current_val = current_val_raw - current_bias;

    return current_val;
}

static inline float pi_control(const float error)
{
    const float pi_control_p_term = error * CURRENT_CONTROLLER_KP;

    pi_control_i_term += error * CURRENT_CONTROLLER_KI;
    pi_control_i_term = SAT(pi_control_i_term, CURRENT_CONTROLLER_I_TERM_MAX, -CURRENT_CONTROLLER_I_TERM_MAX);
    pi_control_i_term = cla_inputs.enable ? pi_control_i_term : 0.0f;

    const float v_bridge = SAT((pi_control_p_term + pi_control_i_term), V_BRIDGE_MAX, -V_BRIDGE_MAX);

    return v_bridge;
}

static inline float calculate_duty_percent(const float v_bridge)
{
    const float duty_percent = SAT((v_bridge / V_BRIDGE_MAX), 0.95f, 0.05f);

    return duty_percent;
}

static inline void set_counter_compare_and_direction_pin(const float duty_percent, const bool enable, const float override_duty_percent, const uint16_t override_direction, const bool override_enable)
{
    if (override_enable)
    {
        const float override_duty_percent_saturated = SAT(override_duty_percent, 0.98f, 0.0f);
        const float counter_compare = MOTOR_CONTROL_TBPRD - (fabsf(override_duty_percent_saturated) * MOTOR_CONTROL_TBPRD);
        EPwm1Regs.CMPA.bit.CMPA = __mf32toui16r(counter_compare);
        GpioDataRegs.GPADAT.bit.GPIO1 = override_direction;
    }
    else if (enable)
    {
        const uint16_t direction = (duty_percent > 0.0f) ? 1U : 0U;
        const float counter_compare = MOTOR_CONTROL_TBPRD - (fabsf(duty_percent) * MOTOR_CONTROL_TBPRD);
        EPwm1Regs.CMPA.bit.CMPA = __mf32toui16r(counter_compare);
        GpioDataRegs.GPADAT.bit.GPIO1 = direction;
    }
    else
    {
        EPwm1Regs.CMPA.bit.CMPA = MOTOR_CONTROL_TBPRD;
    }
}


__interrupt void sw_task(void)
{
    // initialize global variables
    pi_control_i_term = 0.0f;
    current_bias = 0.0f;
}
