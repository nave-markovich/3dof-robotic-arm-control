clear; clc; close all;

% ============================================================
% 3-DOF robot arm simulation - zero input
% State is in degrees
% ============================================================

% -----------------------------
% Robot real parameters
% -----------------------------

% Link lengths [m]
p.l1 = 144e-3;
p.l2 = 261e-3;
p.l3 = 344e-3;

% Center of mass distances [m]
p.lc1 = p.l1/2;
p.lc2 = p.l2/2;
p.lc3 = p.l3/2;

% Masses [kg]
p.m1 = 2.22;
p.m2 = 1.61;
p.m3 = 1.75;

% Gravity [m/s^2]
p.g = 9.81;

% Inertia tensor values at COM [kg*m^2]
p.Ixx1 = 0.02;
p.Iyy1 = 0.01;
p.Izz1 = 0.01;

p.Ixx2 = 0.01;
p.Iyy2 = 0.01;
p.Izz2 = 0.01;

p.Ixx3 = 0.00;
p.Iyy3 = 0.02;
p.Izz3 = 0.02;

% -----------------------------
% Initial conditions [degrees]
% Start position with zero velocity
% -----------------------------
theta1_0 = 20;     % [deg]
theta2_0 = 30;     % [deg]
theta3_0 = -20;    % [deg]

dtheta1_0 = 0;     % [deg/s]
dtheta2_0 = 0;     % [deg/s]
dtheta3_0 = 0;     % [deg/s]

x0 = [
    theta1_0;
    theta2_0;
    theta3_0;
    dtheta1_0;
    dtheta2_0;
    dtheta3_0
];

% -----------------------------
% Simulation time
% -----------------------------
dt = 0.01;          % [s]
tFinal = 10;        % [s]
tspan = 0:dt:tFinal;

% -----------------------------
% Numerical integration
% -----------------------------
options = odeset( ...
    'RelTol', 1e-7, ...
    'AbsTol', 1e-9, ...
    'MaxStep', dt ...
);

[t, x] = ode45(@(t, x) robotDynamics(t, x, p), tspan, x0, options); 

% -----------------------------
% Plot joint angles [deg]
% -----------------------------
figure;
plot(t, x(:,1), 'LineWidth', 1.5); hold on;
plot(t, x(:,2), 'LineWidth', 1.5);
plot(t, x(:,3), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Joint angles [deg]');
legend('\theta_1', '\theta_2', '\theta_3');
title('Zero-input simulation: joint angles');

angleMax = max(abs(x(:,1:3)), [], 'all');
angleLimit = max(180, ceil(angleMax/30)*30);
ylim([-angleLimit angleLimit]);

% -----------------------------
% Plot joint velocities [deg/s]
% -----------------------------
figure;
plot(t, x(:,4), 'LineWidth', 1.5); hold on;
plot(t, x(:,5), 'LineWidth', 1.5);
plot(t, x(:,6), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Joint velocities [deg/s]');
legend('d\theta_1', 'd\theta_2', 'd\theta_3');
title('Zero-input simulation: joint velocities');

velMax = max(abs(x(:,4:6)), [], 'all');
velLimit = max(100, ceil(velMax/50)*50);
ylim([-velLimit velLimit]);
% -----------------------------
% Stick-figure animation
% -----------------------------
animateRobot(t, x, p);