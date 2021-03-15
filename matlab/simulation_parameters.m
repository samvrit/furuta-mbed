load Matrices.mat

x1_init_deg = 0;
x2_init_deg = 0;
x3_init_deg = 0;

measurement_mean = 0;
measurement_variance = 1e-6; % (rad)

actuator_noise_mean = 0;
actuator_noise_variance = 0.05; % (N-m)

process_noise_mean = 0;
process_noise_variance = 1e-6; % (N-m)

bearings_frictional_damping = 1e-9; % N-m/(rad/s)

Ts = 200e-6; % (s)

% transport_delay_wifi = 100e-6;
transport_delay_wifi = 100e-6;
zero_order_hold_measurement = 500e-6; % seconds -- this is the cycle time of high level controller
zero_order_hold_torque_command = 100e-6;

Q_process = 10;
R_sensor = eye(3) .* measurement_variance;

sys_ss = ss(A_matrix, B_matrix, C_matrix, D_matrix);
dsys = c2d(sys_ss, Ts);

[~, L, P] = kalman(dsys, Q_process, R_sensor);