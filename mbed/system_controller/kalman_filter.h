#include "mbed.h"
#include "arm_math.h"

extern const float32_t A_minus_B_K_f32[36];

extern const float32_t C_f32[18];

extern const float32_t Kalman_gain_f32[18];

extern const float32_t LQR_gain_f32[6];

extern float32_t Xest_f32[6];                              // (A - B*K)*Xest (a priori)
extern float32_t measurement_vector_f32[3];                // z
extern float32_t measurement_prediction_vector_f32[3];     // C*Xest
extern float32_t measurement_error_vector_f32[3];          // z - C*Xest
extern float32_t correction_vector_f32[6];                 // KalmanGain*(z - C*Xest) (a poteriori)
extern float32_t u_f32[1];                                 // torque command (-K*u)
extern float32_t Xest_prev_f32[6];                         // previous state estimate vector

extern arm_matrix_instance_f32 A_minus_B_K;
extern arm_matrix_instance_f32 C;
extern arm_matrix_instance_f32 Kalman_gain;
extern arm_matrix_instance_f32 LQR_gain;
extern arm_matrix_instance_f32 Xest;
extern arm_matrix_instance_f32 Xest_prev;
extern arm_matrix_instance_f32 measurement_vector;
extern arm_matrix_instance_f32 measurement_prediction_vector;
extern arm_matrix_instance_f32 measurement_error_vector;
extern arm_matrix_instance_f32 correction_vector;
extern arm_matrix_instance_f32 u;

void matrices_init(void);
int compute_a_priori(void);
int compute_a_posteriori(void);
int add_a_priori_a_posteriori(void);
int compute_torque_command(void);

void control_loop(void);