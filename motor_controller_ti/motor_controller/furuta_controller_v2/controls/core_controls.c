// Includes
#include "core_controls.h"
#include "observer_controller.h"
#include "rls_comms.h"
#include "esp_comms.h"
#include "state_machine.h"
#include "fast_logging.h"
#include "motor_control.h"
#include "host_comms.h"
#include "external_switch.h"
#include "matrix_definitions.h"
#include "cpu_cla_shared.h"
#include <string.h>
#include <stdbool.h>

#include "driverlib.h"

// Defines
#define SAT(x, max, min)    ( (x) > (max) ? (max) : ((x) < (min) ? (min) : (x)))

#define MEASUREMENTS_LPF_A (1.0f)

#define TICK_1KHZ_AT_10KHZ (10U)
#define TICK_200HZ_AT_10KHZ (50U)

#define TORQUE_MAX (5.0f)

#define TIMESTEP (1e-4f)    // [s] 10kHz

#define PERIOD_1KHZ (0.001f)
#define PERIOD_200HZ (0.005f)

#define PI (3.1415f)

// Extern data declaration
uint16_t task_200Hz_flag = 0U;
uint16_t task_1kHz_flag = 0U;

// Private data

struct core_controls_data_S
{
    float torque_cmd;
    float measurements[N_STATES];
    float measurements_lpf[N_STATES];
    uint16_t rls_error_bitfield;
    uint16_t motor_fault_flag;
    uint16_t rls_fault_flag;
    bool covariance_matrix_initialized;
    controller_state_E controller_state;
};

struct core_controls_data_S core_controls_data = { 0 };

kf_input_S kf_input;
kf_states_S kf_states;

// Private function declarations
void update_measurements_200Hz(void);

// ISR functions
__interrupt void epwm3ISR(void)
{
    static uint32_t tick_1kHz = 0U;
    static uint32_t tick_200Hz = 0U;

    GPIO_writePin(2U, 1U);

    if(++tick_1kHz == TICK_1KHZ_AT_10KHZ)
    {
        tick_1kHz = 0U;
        task_1kHz_flag = 1U;

#if (SINGLE_PENDULUM)
        rls_get_position(&core_controls_data.measurements[0], &core_controls_data.measurements[2], &core_controls_data.rls_error_bitfield, PERIOD_1KHZ);
#else
        rls_get_position(&core_controls_data.measurements[0], &core_controls_data.measurements[3], &core_controls_data.rls_error_bitfield, PERIOD_1KHZ);
#endif

        core_controls_data.motor_fault_flag = !GPIO_readPin(123);  // active low

        core_controls_data.rls_fault_flag = (core_controls_data.rls_error_bitfield > 0U);

        const bool fault_present = core_controls_data.motor_fault_flag || core_controls_data.rls_fault_flag;

        core_controls_data.controller_state = state_machine_step(core_controls_data.measurements, fault_present);
    }

    if(++tick_200Hz == TICK_200HZ_AT_10KHZ)
    {
        tick_200Hz = 0U;
        task_200Hz_flag = 1U;
        update_measurements_200Hz();
    }

    #pragma UNROLL(6)
    for (uint16_t i = 0; i < N_STATES; i++)
    {
        core_controls_data.measurements_lpf[i] += (core_controls_data.measurements[i] - core_controls_data.measurements_lpf[i]) * MEASUREMENTS_LPF_A;
    }

    const bool enable = core_controls_data.covariance_matrix_initialized && (CONTROLLER_ACTIVE == core_controls_data.controller_state);

    kf_observer_step(core_controls_data.measurements_lpf, enable, &kf_input, &kf_states);

    const float torque_cmd_from_controller = kf_control_output(kf_states.x_hat, TIMESTEP, &kf_input);

    core_controls_data.torque_cmd = enable ? SAT(torque_cmd_from_controller, TORQUE_MAX, -TORQUE_MAX) : 0.0f;

    cla_inputs.enable = enable && (host_rx_command_motor_enable || motor_enable_switch);

    cla_inputs.torque_cmd = core_controls_data.torque_cmd;

    GPIO_writePin(2U, 0U);

    EPWM_clearEventTriggerInterruptFlag(EPWM3_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

// Private functions
void update_measurements_200Hz(void)
{
#if (SINGLE_PENDULUM)
    float dummy_measurement1 = 0.0f;
    float dummy_measurement2 = 0.0f;

    esp_get_data(&core_controls_data.measurements[1], &dummy_measurement1, &core_controls_data.measurements[3], &dummy_measurement2, PERIOD_200HZ);
#else
    esp_get_data(&core_controls_data.measurements[1], &core_controls_data.measurements[2], &core_controls_data.measurements[4], &core_controls_data.measurements[5], PERIOD_200HZ);
#endif
}

// Public Functions

void controller_init(void)
{
    memcpy(kf_input.A, A, N_STATES*N_STATES*sizeof(float));
    memcpy(kf_input.B, B, N_STATES*N_STATES*sizeof(float));
    memcpy(kf_input.C, C, N_STATES*N_STATES*sizeof(float));
    memcpy(kf_input.K, K, N_STATES*N_STATES*sizeof(float));

    kf_input.Q = Q;
    kf_input.R = R;

    kf_input.timestep = TIMESTEP;

    kf_observer_init(&kf_input, &kf_states);

    for(int i = 0; i < 20000; i++)
    {
        core_controls_data.covariance_matrix_initialized = kf_covariance_matrix_step(&kf_input, &kf_states);
    }

    GPIO_writePin(52U, core_controls_data.covariance_matrix_initialized);
}

// Getter functions

void controller_get_measurements(float measurements_output[3])
{
    for (uint16_t i = 0; i < 3; i++)
    {
        measurements_output[i] = core_controls_data.measurements_lpf[i];
    }
}

void controller_get_state_estimate(float x_hat_output[N_STATES])
{
    for(uint16_t i = 0; i < N_STATES; i++)
    {
        x_hat_output[i] = kf_states.x_hat[i];
    }
}

uint16_t controller_get_rls_error_bitfield(void)
{
    return core_controls_data.rls_error_bitfield;
}

uint16_t controller_get_motor_fault(void)
{
    return core_controls_data.motor_fault_flag;
}

float controller_get_torque_cmd(void)
{
    return core_controls_data.torque_cmd;
}

uint16_t controller_get_controller_state(void)
{
    return (uint16_t)core_controls_data.controller_state;
}

