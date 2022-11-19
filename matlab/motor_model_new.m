close all; clear all; clc;

Kt = 0.968; % Nm/A
Kv = 1.7103; % V/rad/s
L = 2.3e-3; % H
R = 2.18; % ohms
J = 12e-9; % kgm^2
B = 0.0151; % Nm/rad/s

s = tf('s');
motor_tf = (s*J+B)/((s*L+R)*(s*J+B) + Kt*Kv);

Kp = 46.576;
Ki = 2352902.968;

pi_tf = (s*Kp + Ki)/s;

closed_loop_tf = (pi_tf*motor_tf) / (1 + motor_tf*pi_tf);

disturbance_tf = (motor_tf) / (1 + motor_tf*pi_tf);

noise_sensitivity_tf = 1 / (1 + motor_tf*pi_tf);

foa_tf = 1/(5e-5*s + 1);

figure
step(closed_loop_tf, foa_tf);
title('Closed Loop Step Response')
legend('closed loop', 'first order approx')

figure
step(disturbance_tf)
title('Disturbance Step')

figure
step(noise_sensitivity_tf)
title('Noise sensitivity step')
