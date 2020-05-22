#include "kalman_filter.h"
#include "acquire_feedback.h"

EventQueue control_queue(32 * EVENTS_EVENT_SIZE);

const float32_t A_minus_B_K_f32[36] = 
{
    1.0000,    0.0001,    0.0003,    0.0001,    0.0000,    0.0000,
   -0.0000,    1.0001,    0.0003,   -0.0000,    0.0001,    0.0000,
    0.0000,   -0.0001,    0.9997,    0.0000,   -0.0000,    0.0001,
   -0.0466,    1.8160,    5.1239,    0.9133,    0.6802,    0.7570,
   -0.0533,    2.0811,    5.8598,   -0.0992,    1.7782,    0.8660,
    0.0582,   -2.2760,   -6.4036,    0.1084,   -0.8511,    0.0529
};

const float32_t C_f32[18] =
{
     1,     0,     0,     0,     0,     0,
     0,     1,     0,     0,     0,     0,
     0,     0,     1,     0,     0,     0
};

const float32_t Kalman_gain_f32[18] = 
{
    0.0205,    0.0223,   -0.0241,
    0.0223,    0.0263,   -0.0277,
   -0.0241,   -0.0277,    0.0315,
    7.2089,    8.2396,   -9.0073,
    8.2435,    9.4313,  -10.3079,
   -9.0136,  -10.3125,   11.2827
};

const float32_t LQR_gain_f32[6] = 
{
    3.1623, -122.7917, -348.0623, 5.8876, -46.2046, -51.4185
};

float32_t Xest_f32[6];
float32_t Xest_prev_f32[6];
float32_t measurement_vector_f32[3];
float32_t measurement_prediction_vector_f32[3];
float32_t measurement_error_vector_f32[3];
float32_t correction_vector_f32[6];
float32_t u_f32[1];

arm_matrix_instance_f32 A_minus_B_K;
arm_matrix_instance_f32 C;
arm_matrix_instance_f32 Kalman_gain;
arm_matrix_instance_f32 LQR_gain;
arm_matrix_instance_f32 Xest;
arm_matrix_instance_f32 Xest_prev;
arm_matrix_instance_f32 measurement_vector;
arm_matrix_instance_f32 measurement_prediction_vector;
arm_matrix_instance_f32 measurement_error_vector;
arm_matrix_instance_f32 correction_vector;
arm_matrix_instance_f32 u;

void matrices_init(void)
{
    uint32_t srcRows, srcColumns;

    srcRows = 6;
    srcColumns = 6;
    arm_mat_init_f32(&A_minus_B_K, srcRows, srcColumns, (float32_t *)A_minus_B_K_f32);

    srcRows = 3;
    srcColumns = 6;
    arm_mat_init_f32(&C, srcRows, srcColumns, (float32_t *)C_f32);

    srcRows = 6;
    srcColumns = 3;
    arm_mat_init_f32(&Kalman_gain, srcRows, srcColumns, (float32_t *)Kalman_gain_f32);

    srcRows = 1;
    srcColumns = 6;
    arm_mat_init_f32(&LQR_gain, srcRows, srcColumns, (float32_t *)LQR_gain_f32);

    srcRows = 6;
    srcColumns = 1;
    arm_mat_init_f32(&Xest, srcRows, srcColumns, (float32_t *)Xest_f32);

    srcRows = 6;
    srcColumns = 1;
    arm_mat_init_f32(&Xest_prev, srcRows, srcColumns, (float32_t *)Xest_prev_f32);

    srcRows = 3;
    srcColumns = 1;
    arm_mat_init_f32(&measurement_vector, srcRows, srcColumns, (float32_t *)measurement_vector_f32);

    srcRows = 3;
    srcColumns = 1;
    arm_mat_init_f32(&measurement_prediction_vector, srcRows, srcColumns, (float32_t *)measurement_prediction_vector_f32);

    srcRows = 3;
    srcColumns = 1;
    arm_mat_init_f32(&measurement_error_vector, srcRows, srcColumns, (float32_t *)measurement_error_vector_f32);

    srcRows = 6;
    srcColumns = 1;
    arm_mat_init_f32(&correction_vector, srcRows, srcColumns, (float32_t *)correction_vector_f32);

    srcRows = 1;
    srcColumns = 1;
    arm_mat_init_f32(&u, srcRows, srcColumns, (float32_t *)u_f32);

}

int compute_a_priori(void)
{
    const arm_status status = arm_mat_mult_f32(&A_minus_B_K, &Xest_prev, &Xest);

    if(ARM_MATH_SUCCESS == status)
    {
        return 1;
    }
    else return 0;
}

int compute_a_posteriori(void)
{
    const arm_status status1 = arm_mat_mult_f32(&C, &Xest_prev, &measurement_prediction_vector);
    const arm_status status2 = arm_mat_sub_f32(&measurement_vector, &measurement_prediction_vector, &measurement_error_vector);
    const arm_status status3 = arm_mat_mult_f32(&Kalman_gain, &measurement_error_vector, &correction_vector);

    if((ARM_MATH_SUCCESS == status1) && (ARM_MATH_SUCCESS == status2) && (ARM_MATH_SUCCESS == status3))
    {
        return 1;
    }
    else return 0;
}

int add_a_priori_a_posteriori(void)
{
    const arm_status status = arm_mat_add_f32(&Xest, &correction_vector, &Xest);

    if(ARM_MATH_SUCCESS == status)
    {
        return 1;
    }
    else return 0;
}

int compute_torque_command(void)
{
    const arm_status status = arm_mat_mult_f32(&LQR_gain, &Xest, &u);
    *u_f32 *= -1.0f;    // multiply with -1 because equation is u = -K*x

    if(ARM_MATH_SUCCESS == status)
    {
        return 1;
    }
    else return 0;
}

void control_loop(void)
{
    osEvent evt = feedback_queue.get(0);
    if (evt.status == osEventMessage)
    {
        state_vector *X = (state_vector*)evt.value.p;


        mpool.free(X);
    }
}
