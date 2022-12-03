
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "kalman_filter_lqr_lib/observer_controller.h"
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 6
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */

kf_input_S kf_input;
kf_states_S kf_states;

float torque_cmd = 0.0f;
bool inverse_valid = true;
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
void kf_and_controller_Start_wrapper(real_T *xD)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Custom Start code goes here.
 */
const float A[N_STATES][N_STATES] = {	{0.000000,	0.000000,	0.000000,	1.000000,	0.000000,	0.000000},
										{0.000000,	0.000000,	0.000000,	0.000000,	1.000000,	0.000000},
										{0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	1.000000},
										{0.000000,	82.411254,	-2.785977,	0.000000,	0.000000,	0.000000},
										{0.000000,	130.127738,	-22.802044,	0.000000,	0.000000,	0.000000},
										{0.000000,	-142.150128,	76.102558,	0.000000,	0.000000,	0.000000}};

const float B[N_STATES][N_STATES] = {	{0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000},
										{0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000},
										{0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000},
										{147.219505,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000},
										{168.419459,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000},
										{-184.196878,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000}};

const float C[N_STATES][N_STATES] = {	{1.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000},
										{0.000000,	1.000000,	0.000000,	0.000000,	0.000000,	0.000000},
										{0.000000,	0.000000,	1.000000,	0.000000,	0.000000,	0.000000},
										{0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000},
										{0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000},
										{0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000}};

const float K[N_STATES][N_STATES] = {	{3.1623,    -1.1589e3,    -3.2336e3,    35.0927,  -429.5221,  -464.7969},
										{0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000},
										{0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000},
										{0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000},
										{0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000},
										{0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000}};


const float Q = 7.5e-5f;
const float R = 1.21e-6f;

memcpy(kf_input.A, A, N_STATES*N_STATES*sizeof(float));
memcpy(kf_input.B, B, N_STATES*N_STATES*sizeof(float));
memcpy(kf_input.C, C, N_STATES*N_STATES*sizeof(float));
memcpy(kf_input.K, K, N_STATES*N_STATES*sizeof(float));

kf_input.Q = Q;
kf_input.R = R;
kf_input.timestep = 5e-5f;

observer_init(&kf_input, &kf_states);

for(int i = 0; i < 20000; i++)
{
    inverse_valid = covariance_matrix_step(&kf_input, &kf_states);
    if (!inverse_valid)
    {
        break;
    }
}
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
void kf_and_controller_Outputs_wrapper(const real_T *u0,
			real_T *y0,
			const real_T *xD)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
/* This sample sets the output equal to the input
      y0[0] = u0[0]; 
 For complex signals use: y0[0].re = u0[0].re; 
      y0[0].im = u0[0].im;
      y1[0].re = u1[0].re;
      y1[0].im = u1[0].im;
 */

y0[0] = torque_cmd;
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Updates function
 *
 */
void kf_and_controller_Update_wrapper(const real_T *u0,
			real_T *y0,
			real_T *xD)
{
/* %%%-SFUNWIZ_wrapper_Update_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Code example
 *   xD[0] = u0[0];
 */

const float measurement[6] = {u0[0], u0[1], u0[2], u0[3], u0[4], u0[5]};

static float measurement_lpf[6] = {0};

for (int i = 0; i < 6; i++)
{
    measurement_lpf[i] += (measurement[i] - measurement_lpf[i]) * 0.09090909091f;
}

// covariance_matrix_step();
observer_step(measurement_lpf, inverse_valid, &kf_input, &kf_states);

torque_cmd = control_output(&(kf_states.x_hat), kf_input.timestep, &kf_input);
/* %%%-SFUNWIZ_wrapper_Update_Changes_END --- EDIT HERE TO _BEGIN */
}

