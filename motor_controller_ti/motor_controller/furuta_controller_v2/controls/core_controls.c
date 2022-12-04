// Includes
#include "core_controls.h"
#include "observer_controller.h"
#include <string.h>

#include "driverlib.h"


// Private data

const float Ts = 1e-4f;
float torque_cmd = 0.0f;

// Public Functions

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


__interrupt void epwm3ISR(void)
{
    GPIO_writePin(2, 1);

    const float measurement[6] = { 0 };
    kf_observer_step(measurement, true, &kf_input, &kf_states);

    torque_cmd = kf_control_output(kf_states.x_hat, Ts, &kf_input);

    GPIO_writePin(2, 0);

    EPWM_clearEventTriggerInterruptFlag(EPWM3_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

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
