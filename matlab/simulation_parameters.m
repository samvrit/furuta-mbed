load Matrices.mat

x1_init_deg = 0;
x2_init_deg = 2;
x3_init_deg = 3;

measurement_mean = 0;
measurement_variance = 0.0001;

Ts = 100e-6;

transport_delay_wifi = 5e-6;
zero_order_hold_measurement = Ts; % seconds -- this is the cycle time of high level controller
zero_order_hold_torque_command = 100e-6;

Q_process = 1;
R_sensor = eye(6) .* 0.0001;

C_matrix = diag([1, 1, 1, 1, 1, 1]);
sys_ss = ss(A_matrix, B_matrix, C_matrix, D_matrix);
dsys = c2d(sys_ss, Ts);

[~, L, ~] = kalman(dsys, Q_process, R_sensor);