clear; clc; close all;

% ============================================================
% 3-DOF robot arm simulation - attractive potential control
% The user enters a Cartesian target point [x y z] in meters.
% The target is converted to joint angles using inverse kinematics.
% State is stored in degrees.
% Dynamic calculations are performed in radians.
% Existing zero-input functions are not modified.
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
theta0_deg = [
    20;
    30;
   -20
];

dtheta0_deg_s = [
    0;
    0;
    0
];

x0 = [
    theta0_deg;
    dtheta0_deg_s
];

% -----------------------------
% Joint limits [degrees]
% Replace these values by the real mechanical limits if known.
% The initial configuration and the IK target are checked before simulation.
% -----------------------------
jointLimits_deg = [
   -180, 180;
    -90,  90;
   -150, 150
];

assertJointTargetInRange(theta0_deg, jointLimits_deg, 'initial configuration');

% -----------------------------
% Reachable workspace information
% The reachable workspace is checked as a radius from the shoulder point.
% The shoulder point is the end of link 1: [0; l1; 0].
% The allowed radial range is abs(l3 - l2) <= r <= l2 + l3.
% -----------------------------
workspaceCenter_m = [0; p.l1; 0];

minReach_m = abs(p.l3 - p.l2);
maxReach_m = p.l2 + p.l3;

defaultTargetPoint_m = [0.25;0.45;0.20];

fprintf('\nReachable workspace information:\n');
fprintf('Workspace center = [%.4f, %.4f, %.4f] [m]\n', ...
    workspaceCenter_m(1), workspaceCenter_m(2), workspaceCenter_m(3));

fprintf('Minimum allowed radius = %.4f [m]\n', minReach_m);
fprintf('Maximum allowed radius = %.4f [m]\n', maxReach_m);

fprintf('Example valid target point: [%.4f %.4f %.4f] [m]\n\n', ...
    defaultTargetPoint_m(1), defaultTargetPoint_m(2), defaultTargetPoint_m(3));


targetInputStr = strtrim(input('Enter Cartesian target [x y z] in meters, or press Enter for default: ', 's'));

if isempty(targetInputStr)
    targetPoint_m = defaultTargetPoint_m;
else
    cleanedTargetInput = regexprep(targetInputStr, '[\[\];,]', ' ');
    targetValues = sscanf(cleanedTargetInput, '%f');

    if numel(targetValues) ~= 3
        error('Invalid Cartesian target. Enter exactly three numeric values, for example: [0.25 0.45 0.20]');
    end

    targetPoint_m = targetValues(:);
end

% -----------------------------
% Convert Cartesian target to joint target using inverse kinematics.
% The selected IK branch is the valid branch closest to theta0_deg.
% -----------------------------
[thetaGoal_deg, ikInfo] = inverseKinematics3DOF(targetPoint_m, p, jointLimits_deg, theta0_deg);

assertJointTargetInRange(thetaGoal_deg, jointLimits_deg, 'target configuration');

goalPoints = robotPoints(thetaGoal_deg, p);
goalPointFromIK_m = goalPoints(:, end);
cartesianError_m = norm(goalPointFromIK_m - targetPoint_m(:));

fprintf('\nSelected Cartesian target [m]: x = %.4f, y = %.4f, z = %.4f\n', ...
    targetPoint_m(1), targetPoint_m(2), targetPoint_m(3));

fprintf('Converted joint target [deg]: theta1 = %.3f, theta2 = %.3f, theta3 = %.3f\n', ...
    thetaGoal_deg(1), thetaGoal_deg(2), thetaGoal_deg(3));

fprintf('Selected IK branch: %d\n', ikInfo.selectedIndex);
fprintf('Forward-kinematics target check error = %.6e [m]\n\n', cartesianError_m);

% -----------------------------
% Controller design
% tau = G(q) - grad(Uatt) - Kd*dq
% Uatt = 0.5*(q - qd)'*Katt*(q - qd)
% q and dq are in radians inside the controller.
% -----------------------------
ctrl.thetaGoal_deg = thetaGoal_deg;
ctrl.targetPoint_m = targetPoint_m;
ctrl.jointLimits_deg = jointLimits_deg;
ctrl.useGravityCompensation = true;

% Gain design by approximate second-order behavior around the target.
% Ts is the approximate settling time [s].
% zeta = 1 gives approximately critically damped behavior.
Ts = [
    2.0;
    2.0;
    2.0
];

zeta = [
    1.0;
    1.0;
    1.0
];

Mgoal = inertiaMatrix(deg2rad(thetaGoal_deg), p);
Ieff = max(diag(Mgoal), 1e-6);

omega_n = 4 ./ (zeta .* Ts);

ctrl.Katt = diag(Ieff .* omega_n.^2);              % [N*m/rad]
ctrl.Kd   = diag(2 .* zeta .* Ieff .* omega_n);    % [N*m*s/rad]

% Store controller design data for later inspection.
ctrl.gainDesign.Ts = Ts;
ctrl.gainDesign.zeta = zeta;
ctrl.gainDesign.omega_n = omega_n;
ctrl.gainDesign.Mgoal = Mgoal;
ctrl.gainDesign.Ieff = Ieff;
ctrl.gainDesign.thetaGoal_deg = thetaGoal_deg;
ctrl.gainDesign.KattFormula = 'Katt = diag(Ieff .* omega_n.^2)';
ctrl.gainDesign.KdFormula = 'Kd = diag(2 .* zeta .* Ieff .* omega_n)';
ctrl.gainDesign.omegaFormula = 'omega_n = 4 ./ (zeta .* Ts)';

% Optional torque saturation.
% Keep disabled unless a physical motor torque limit is required.
ctrl.useTorqueSaturation = true;
ctrl.tauMax = [20;20;20];

% Event stopping tolerances.
ctrl.goalTolerance_deg = 1e-3;
ctrl.velocityTolerance_deg_s = 1e-3;

fprintf('Katt [N*m/rad]:\n');
disp(ctrl.Katt);

fprintf('Kd [N*m*s/rad]:\n');
disp(ctrl.Kd);

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
    'MaxStep', dt, ...
    'Events', @(t, x) goalReachedEvent(t, x, ctrl) ...
);

[t, x, te, ~, ~] = ode45(@(t, x) robotDynamicsPotential(t, x, p, ctrl), ...
    tspan, x0, options);

if ~isempty(te)
    fprintf('Target reached at t = %.4f [s].\n', te(end));
else
    finalError = norm(x(end, 1:3).' - thetaGoal_deg);
    fprintf('Simulation ended before full convergence. Final angle error norm = %.6f [deg].\n', ...
        finalError);
end

% -----------------------------
% Compute control torque, potential, and end-effector history
% -----------------------------
tauHistory = zeros(length(t), 3);
UHistory = zeros(length(t), 1);
endEffectorHistory = zeros(length(t), 3);

for k = 1:length(t)
    [~, tau_k, U_k] = robotDynamicsPotential(t(k), x(k, :).', p, ctrl);
    tauHistory(k, :) = tau_k.';
    UHistory(k) = U_k;

    points_k = robotPoints(x(k, 1:3).', p);
    endEffectorHistory(k, :) = points_k(:, end).';
end
cartesianErrorHistory = zeros(length(t), 1);

for k = 1:length(t)
    cartesianErrorHistory(k) = norm(endEffectorHistory(k, :).' - targetPoint_m(:));
end

figure;
semilogy(t, cartesianErrorHistory, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Cartesian position error [m]');
title('End-effector Cartesian position error');
% -----------------------------
% Plot joint angles [deg]
% -----------------------------
figure;
plot(t, x(:,1), 'LineWidth', 1.5); hold on;
plot(t, x(:,2), 'LineWidth', 1.5);
plot(t, x(:,3), 'LineWidth', 1.5);

plot(t, thetaGoal_deg(1)*ones(size(t)), '--', 'LineWidth', 1.0);
plot(t, thetaGoal_deg(2)*ones(size(t)), '--', 'LineWidth', 1.0);
plot(t, thetaGoal_deg(3)*ones(size(t)), '--', 'LineWidth', 1.0);

grid on;
xlabel('Time [s]');
ylabel('Joint angles [deg]');
legend('\theta_1', '\theta_2', '\theta_3', ...
       '\theta_{1d}', '\theta_{2d}', '\theta_{3d}', ...
       'Location', 'best');
title('Potential-control simulation: joint angles');

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
legend('d\theta_1', 'd\theta_2', 'd\theta_3', 'Location', 'best');
title('Potential-control simulation: joint velocities');

% -----------------------------
% Plot control torques [N*m]
% -----------------------------
figure;
plot(t, tauHistory(:,1), 'LineWidth', 1.5); hold on;
plot(t, tauHistory(:,2), 'LineWidth', 1.5);
plot(t, tauHistory(:,3), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Control torque [N*m]');
legend('\tau_1', '\tau_2', '\tau_3', 'Location', 'best');
title('Potential-control simulation: control torques');

% -----------------------------
% Plot attractive potential energy
% -----------------------------
figure;
plot(t, UHistory, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('U_{att} [J]');
title('Attractive potential energy');

% -----------------------------
% Plot end-effector Cartesian position [m]
% -----------------------------
figure;
plot(t, endEffectorHistory(:,1), 'LineWidth', 1.5); hold on;
plot(t, endEffectorHistory(:,2), 'LineWidth', 1.5);
plot(t, endEffectorHistory(:,3), 'LineWidth', 1.5);

plot(t, targetPoint_m(1)*ones(size(t)), '--', 'LineWidth', 1.0);
plot(t, targetPoint_m(2)*ones(size(t)), '--', 'LineWidth', 1.0);
plot(t, targetPoint_m(3)*ones(size(t)), '--', 'LineWidth', 1.0);

grid on;
xlabel('Time [s]');
ylabel('End-effector position [m]');
legend('x', 'y', 'z', 'x_d', 'y_d', 'z_d', 'Location', 'best');
title('Potential-control simulation: end-effector position');

% -----------------------------
% Stick-figure animation
% -----------------------------
animateRobotPotential(t, x, p, thetaGoal_deg, targetPoint_m);
% % -----------------------------
% % Export stick-figure simulation to MP4
% % -----------------------------
% 
% targetPoints_m = robotPoints(thetaGoal_deg, p);
% targetPoint_m = targetPoints_m(:, end);
% 
% exportVideo = true;
% 
% if exportVideo
%     exportRobotSimulationMP4(t, x, p, thetaGoal_deg, targetPoint_m, 'robot_potential_simulation.mp4');
% end
% fprintf('\nSimulation and MP4 export finished successfully.\n');

% -----------------------------
% Stick-figure animation
% Set runAnimation inside animateRobotPotential.m to false if you do not
% want the Command Window to be occupied by the animation.
% -----------------------------
% runAnimationAfterExport = false;
% 
% if runAnimationAfterExport
%     animateRobotPotential(t, x, p, thetaGoal_deg, targetPoint_m);
% end

