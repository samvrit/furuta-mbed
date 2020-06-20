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

% mass_data = [23; 62; 108]; % grams
% current_data = [0.13; 0.4; 0.57]; % Amperes
% arm_length = 0.447; %m

% Determine K using linear regression
Kt = current_data \ ((1e-3 * 9.81 * arm_length) .* mass_data);

%% Frictional damping coefficient calculation
% Experiment description:
% At no load and steady state, the mechanical dynamics become b = (Kt*i / angular_velocity)
% Hence, freely run the motor and record the winding current and angular
% velocity using the encoder data on a scope
pulses_per_revolution = 480; % 16 rising edges * gear ratio (30)
pulse_interval = 331e-6; % delta t in seconds between two rising edges
average_current = 265e-3; % amperes
angular_velocity = (2*pi)/((pulses_per_revolution)*pulse_interval); % rad/s
b = (Kt * average_current) / angular_velocity;

%% Experimental Transfer function
% Experiment setup:
% - Connect current sensor in series with motor and battery
% - Connect one channel of oscilloscope to the current sensor output
% - Connect another channel of oscilloscope to the battery terminals
% - Setup a rising edge trigger for the battery channel
% - This will give the step response for the current/voltage transfer
% function, which is a first order transfer function
% - measure rising time, gain and delay if any

rising_time = 1.3e-3; % seconds
battery_step_amplitude = 12.27; % volts
current_amplitude_at_steady_state = 8.46; % amperes
gain = current_amplitude_at_steady_state / battery_step_amplitude;

rotor_mass = 30e-3; % kg
rotor_radius = 11.5e-3; %m
gear_ratio = 30;
rotor_inertia = 0.5 * rotor_mass * (rotor_radius^2) * (gear_ratio^2); % moment of inertia measured at output of gearbox

J = 0.0032 + rotor_inertia; % moment of inertia of load (link 1) in kg m^2
R = 1/gain;
Kv = 0.2915; % measured using actual data and regression analysis
L = R*rising_time;  % at no load, rise time = L/R

%% Motor transfer function
num = (Kt/L).*[1 (b/J)];
den = [1 ((R*J + b*L)/(L*J)) ((R*b + Kv*Kt)/(L*J))];
torque_tf = tf(num, den);

%% Motor state space representation
Ts_motor = 20e-6; % time step in seconds for motor controller
A_motor = [-R/L -Kv/L; Kt/J -b/J];
B_motor = [1/L; 0];
C_motor = [1 0; 0 1];
D_motor = [0; 0];
motor_ss = ss(A_motor, B_motor, C_motor, D_motor);
motor_ss_discrete = c2d(motor_ss, Ts_motor);
[Gain_motor, ~] = lqr(A_motor, B_motor, diag([100000 0]), 1);

%% PI Controller
opts = pidtuneOptions('CrossoverFrequency',1.23e4,'PhaseMargin',88);
[C, info] = pidtune(torque_tf, 'PI', opts);

disp(C.Kp)
disp(C.Ki)
disp(C.Kd)
disp(info)

closed_loop_tf = (C*torque_tf) / (1 + (C*torque_tf));

%% Simulation parameters
motor_supply_voltage = 12;
torque_ref = timeseries([-1 1 -1 1 -1], [0.1 0.2 0.4 0.8 0.9]);
zero_order_hold_adc = 20e-6; % seconds -- this is the ADC sampling period