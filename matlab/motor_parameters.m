K = 0.025;
J = 0.01;
b = 0.5;
R = 2;
L = 0.05;

Kp = 1;
Ki = 1;
Kd = 1;

pid_controller = pid(Kp,Ki,Kd);
C = tf(pid_controller);
motor_sys = tf(b/(b*L + K*J), [1 R*b/(b*L + K*J)]);
open_loop = motor_sys*K;

closed_loop = (C*motor_sys) / (1 + (C*motor_sys*K));
