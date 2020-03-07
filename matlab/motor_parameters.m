K = 0.025;
J = 9.9e-5;
b = 1;
R = 1.52;    % calculated by measuring current using a multimeter in series with the motor with no load when 12.39V was applied. Measured current was 0.112A
L = 0.05;

torque_ref = timeseries([0.01 0.02 -0.05 -0.03 0.04], [0.1 0.2 0.4 0.8 0.9]);

motor_sys = tf(1/R, [(b*L + K*J)/(R*b) 1]);
open_loop = motor_sys*K;

opts = pidtuneOptions('CrossoverFrequency',400,'PhaseMargin',90);
[C, info] = pidtune(motor_sys, 'pid', opts);

closed_loop = (C*motor_sys) / (1 + (C*motor_sys*K));