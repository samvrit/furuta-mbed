// Includes
#include "core_controls.h"
#include "observer_controller.h"
#include "rls_comms.h"
#include "esp_comms.h"
#include "host_comms.h"
#include "state_machine.h"
#include <string.h>

#include "driverlib.h"

// Defines

#define TICK_1KHZ_AT_10KHZ (10U)
// Private data

const float Ts = 1e-4f;
float torque_cmd = 0.0f;
float measurements[N_STATES] = {0.0f};
uint16_t rls_error_bitfield = 0U;

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
void update_measurements(void);

// ISR functions

__interrupt void epwm3ISR(void)
{
    static uint32_t tick_1kHz = 0U;

    controller_state_E controller_state;
    uint16_t motor_fault_flag;

    if(++tick_1kHz == TICK_1KHZ_AT_10KHZ)
    {
        tick_1kHz = 0U;
        update_measurements();

        controller_state = state_machine_step(measurements);

        motor_fault_flag = !GPIO_readPin(123);  // active low

        send_data_to_host(kf_states.x_hat, measurements, torque_cmd, (uint16_t)controller_state, rls_error_bitfield, motor_fault_flag);
    }

    const bool enable = (CONTROLLER_ACTIVE == controller_state) && (!motor_fault_flag);

    kf_observer_step(measurements, enable, &kf_input, &kf_states);

    const float torque_cmd_from_controller = kf_control_output(kf_states.x_hat, Ts, &kf_input);

    torque_cmd = enable ? torque_cmd_from_controller : 0.0f;

    EPWM_clearEventTriggerInterruptFlag(EPWM3_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

// Private functions

void update_measurements(void)
{
    float esp_position1 = 0.0f;
    float esp_position2 = 0.0f;

    const float rls_position = rls_get_position(&rls_error_bitfield);
    esp_get_data(&esp_position1, &esp_position2);

    measurements[0] = rls_position;
    measurements[1] = esp_position1;
    measurements[2] = esp_position2;
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

    kf_input.timestep = Ts;

    kf_observer_init(&kf_input, &kf_states);

    for(int i = 0; i < 20000; i++)
    {
        kf_covariance_matrix_step(&kf_input, &kf_states);
    }
}

void controller_get_measurements(float measurements_output[3])
{
    for (uint16_t i = 0; i < 3; i++)
    {
        measurements_output[i] = measurements[i];
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
    return rls_error_bitfield;
}

