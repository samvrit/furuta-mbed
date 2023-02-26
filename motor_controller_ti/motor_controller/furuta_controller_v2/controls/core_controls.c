// Includes
#include "core_controls.h"
#include "observer_controller.h"
#include "rls_comms.h"
#include "esp_comms.h"
#include "state_machine.h"
#include "fast_logging.h"
#include <string.h>

#include "driverlib.h"

// Defines
#define MEASUREMENTS_LPF_A (0.09090909091f)

#define TICK_1KHZ_AT_10KHZ (5U)
#define TICK_100HZ_AT_10KHZ (100U)
#define TICK_5KHZ_AT_10KHZ (2U)

#define TIMESTEP (1e-4f)    // [s] 10kHz

// Extern data declaration
uint16_t task_100Hz_flag = 0U;
uint16_t task_5kHz_flag = 0U;

// Private data

struct core_controls_data_S
{
    float torque_cmd;
    float measurements[N_STATES];
    float measurements_lpf[N_STATES];
    uint16_t rls_error_bitfield;
    uint16_t motor_fault_flag;
    controller_state_E controller_state;
};

struct core_controls_data_S core_controls_data = { 0 };

kf_input_S kf_input;
kf_states_S kf_states;

const float A[N_STATES][N_STATES] = {   {0.000000,  0.000000,   0.000000,   1.000000,   0.000000,   0.000000},
                                        {0.000000,  0.000000,   0.000000,   0.000000,   1.000000,   0.000000},
                                        {0.000000,  0.000000,   0.000000,   0.000000,   0.000000,   1.000000},
                                        {0.000000,  82.411254,  -2.785977,  0.000000,   0.000000,   0.000000},
                                        {0.000000,  130.127738, -22.802044, 0.000000,   0.000000,   0.000000},
                                        {0.000000,  -142.150128,    76.102558,  0.000000,   0.000000,   0.000000}};

const float B[N_STATES][N_STATES] = {   {0.000000,  0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
                                        {0.000000,  0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
                                        {0.000000,  0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
                                        {147.219505,    0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
                                        {168.419459,    0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
                                        {-184.196878,   0.000000,   0.000000,   0.000000,   0.000000,   0.000000}};

const float C[N_STATES][N_STATES] = {   {1.000000,  0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
                                        {0.000000,  1.000000,   0.000000,   0.000000,   0.000000,   0.000000},
                                        {0.000000,  0.000000,   1.000000,   0.000000,   0.000000,   0.000000},
                                        {0.000000,  0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
                                        {0.000000,  0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
                                        {0.000000,  0.000000,   0.000000,   0.000000,   0.000000,   0.000000}};

const float K[N_STATES][N_STATES] = {   {3.162278,  -677.257131,    -1339.746051,   33.700353,  -218.398551,    -198.544387},
                                        {0.000000,  0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
                                        {0.000000,  0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
                                        {0.000000,  0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
                                        {0.000000,  0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
                                        {0.000000,  0.000000,   0.000000,   0.000000,   0.000000,   0.000000}};

const float Q = 7.5e-5f;
const float R = 1.21e-6f;

// Private function declarations
void update_measurements_100Hz(void);

// ISR functions

__interrupt void epwm3ISR(void)
{
    static uint32_t tick_1kHz = 0U;
    static uint32_t tick_100Hz = 0U;
    static uint32_t tick_5kHz = 0U;

    GPIO_writePin(2U, 1U);

    if(++tick_1kHz == TICK_1KHZ_AT_10KHZ)
    {
        tick_1kHz = 0U;

        core_controls_data.measurements[0] = rls_get_position(&core_controls_data.rls_error_bitfield);

        core_controls_data.controller_state = state_machine_step(core_controls_data.measurements);

        core_controls_data.motor_fault_flag = !GPIO_readPin(123);  // active low
    }

    if (++tick_5kHz == TICK_5KHZ_AT_10KHZ)
    {
        task_5kHz_flag = 1U;
    }

    if(++tick_100Hz == TICK_100HZ_AT_10KHZ)
    {
        tick_100Hz = 0U;
        task_100Hz_flag = 1U;
        update_measurements_100Hz();
    }

    #pragma UNROLL(6)
    for (uint16_t i = 0; i < N_STATES; i++)
    {
        core_controls_data.measurements_lpf[i] += (core_controls_data.measurements[i] - core_controls_data.measurements_lpf[i]) * MEASUREMENTS_LPF_A;
    }

    const bool enable = (CONTROLLER_ACTIVE == core_controls_data.controller_state) && (!core_controls_data.motor_fault_flag);

    kf_observer_step(core_controls_data.measurements_lpf, enable, &kf_input, &kf_states);

    const float torque_cmd_from_controller = kf_control_output(kf_states.x_hat, TIMESTEP, &kf_input);

    core_controls_data.torque_cmd = enable ? torque_cmd_from_controller : 0.0f;

    const float logging_signals[2] = { core_controls_data.torque_cmd, (float)core_controls_data.controller_state };

    GPIO_writePin(2U, 0U);

    EPWM_clearEventTriggerInterruptFlag(EPWM3_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

// Private functions

void update_measurements_100Hz(void)
{
    esp_get_data(&core_controls_data.measurements[1], &core_controls_data.measurements[2]);
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
        kf_covariance_matrix_step(&kf_input, &kf_states);
    }
}

// Getter functions

void controller_get_measurements(float measurements_output[3])
{
    for (uint16_t i = 0; i < 3; i++)
    {
        measurements_output[i] = core_controls_data.measurements[i];
    }
}

void controller_get_state_estimate(float x_hat_output[6])
{
    for(uint16_t i = 0; i < 6; i++)
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

