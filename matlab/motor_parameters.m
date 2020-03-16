%% Motor torque constant calculation from experimental data
% Experiment description:
% Setup:
% - Fix an arm of known length to the motor shaft
% - Connect a digital multimeter in series with the motor to measure
% current
% - Place a load cell at the end of the arm to measure force
% - Make sure that the arm is parallel to the ground, so that force at the
% end of the arm is purely vertical
% Test:
% - Connect the battery and measure force and current at steady state

% Data points:
mass_data = [60; 50; 49; 84; 100; 74; 63; 61; 60; 60; 57]; % grams
current_data = [1.2; 1.3; 1.24; 1.15; 1.12; 1.08; 1.03; 1.07; 1.06; 1.05; 1.05]; % Amperes
arm_length = 0.447; %m

% Determine K using linear regression
K = current_data \ ((1e-3 * 9.81 * arm_length) .* mass_data);

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