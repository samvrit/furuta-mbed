Kt = 0.968; % Nm/A
Kv = 1.7103; % V/rad/s
L = 2.3e-3; % H
R = 2.18; % ohms
J = 12e-9; % kgm^2
B = 0.0151; % Nm/rad/s

s = tf('s');
motor_tf = (s*J+B)/((s*L+R)*(s*J+B) + Kt*Kv);

Kp = 615.1057;
Ki = 29615329.4916;

pi_tf = (s*Kp + Ki)/s;

closed_loop_tf = (pi_tf*motor_tf) / (1 + motor_tf*pi_tf);
