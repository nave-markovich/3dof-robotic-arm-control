README.TXT

Zero-Input Robot Arm Simulation

This folder contains the MATLAB files required to simulate the natural motion of a 3-DOF robotic arm when no control input is applied. In this simulation, the robot moves only according to its dynamic model, gravity, inertia, and Coriolis effects.

Important note:
The initial condition of the robot is defined by joint angles, not by a Cartesian point.
This means that the user should change the initial joint configuration theta1, theta2, and theta3, and should not enter or modify an initial Cartesian position.

============================================================
1. Files Included in This Folder
============================================================

1.1 run_zero_input_simulation.m

This is the main script of the zero-input simulation.

The script defines:
- The physical parameters of the robotic arm.
- The initial joint angles.
- The initial joint velocities.
- The simulation time.
- The numerical integration settings.
- The plots of joint angles and joint velocities.
- The animation of the robotic arm motion.

This is the file that should be run by the user.

To run the simulation, open MATLAB, make sure all files are in the same folder, and run:

run_zero_input_simulation

1.2 robotDynamics.m

This function defines the dynamic model of the robotic arm in state-space form.

The state vector is:

x = [theta1; theta2; theta3; dtheta1; dtheta2; dtheta3]

where:
- theta1, theta2, theta3 are the joint angles in degrees.
- dtheta1, dtheta2, dtheta3 are the joint angular velocities in degrees per second.

Inside this function, the angles and velocities are converted from degrees to radians in order to calculate the dynamics.

The simulation is a zero-input simulation, therefore the input torque vector is:

tau = [0; 0; 0]

The equation of motion used in this function is:

M(q)*ddq + C(q,dq)*dq + G(q) = tau

1.3 inertiaMatrix.m

This function calculates the inertia matrix M(q) of the robotic arm.

The input q is given in radians inside this function.

1.4 coriolisMatrix.m

This function calculates the Coriolis and centrifugal matrix C(q,dq) of the robotic arm.

The inputs q and dq are given in radians and radians per second inside this function.

1.5 gravityVector.m

This function calculates the gravity vector G(q) of the robotic arm.

The input q is given in radians inside this function.

1.6 robotPoints.m

This function calculates the Cartesian positions of the robot joints and end-effector for a given joint configuration.

The input q_deg is given in degrees, because the animation uses the simulation state directly.

The output contains the Cartesian points:
- Base point
- End of link 1
- End of link 2
- End of link 3 / end-effector

1.7 animateRobot.m

This function creates a stick-figure animation of the robotic arm motion.

The animation uses the joint angles obtained from the simulation and converts them into Cartesian points using robotPoints.m.

============================================================
2. How to Change the Initial Configuration
============================================================

The initial configuration is changed only in the main script:

run_zero_input_simulation.m

In this section:

theta1_0 = 20;     % [deg]
theta2_0 = 30;     % [deg]
theta3_0 = -20;    % [deg]

the user can change the initial joint angles of the robot.

For example, to start the simulation from:

theta1 = 0 deg
theta2 = 90 deg
theta3 = 0 deg

change the code to:

theta1_0 = 0;      % [deg]
theta2_0 = 90;     % [deg]
theta3_0 = 0;      % [deg]

Important:
These values are joint angles in degrees. They are not Cartesian coordinates.

Do not write an initial point such as:

[x, y, z] = [0.2, 0.4, 0.1]

This simulation does not receive an initial Cartesian point. The robot initial pose is determined only by the joint angles theta1_0, theta2_0, and theta3_0.

============================================================
3. How to Change the Initial Velocity
============================================================

The initial joint velocities are also defined in the main script:

dtheta1_0 = 0;     % [deg/s]
dtheta2_0 = 0;     % [deg/s]
dtheta3_0 = 0;     % [deg/s]

By default, the robot starts from rest.

If the user wants the robot to start with an initial angular velocity, these values can be changed.

For example:

dtheta1_0 = 5;     % [deg/s]
dtheta2_0 = 0;     % [deg/s]
dtheta3_0 = -3;    % [deg/s]

In most standard zero-input tests, the initial velocities should remain zero.

============================================================
4. How to Change the Simulation Time
============================================================

The simulation time is defined in the main script:

dt = 0.01;          % [s]
tFinal = 10;        % [s]
tspan = 0:dt:tFinal;

where:
- dt is the simulation time step.
- tFinal is the final simulation time.
- tspan is the time vector used by ode45.

For example, to run the simulation for 5 seconds, change:

tFinal = 5;

============================================================
5. How the Simulation Works
============================================================

The simulation follows these steps:

1. The robot parameters are defined in run_zero_input_simulation.m.
2. The initial joint angles and velocities are defined.
3. The initial state vector x0 is created.
4. MATLAB solves the dynamic equations using ode45.
5. The joint angles are plotted as a function of time.
6. The joint velocities are plotted as a function of time.
7. The robot motion is animated using animateRobot.m.

============================================================
6. Required File Structure
============================================================

All MATLAB files must be located in the same folder before running the simulation.

The required files are:

- run_zero_input_simulation.m
- robotDynamics.m
- inertiaMatrix.m
- coriolisMatrix.m
- gravityVector.m
- robotPoints.m
- animateRobot.m

If one of these files is missing, MATLAB may not be able to run the simulation correctly.

============================================================
7. Notes About Units
============================================================

In the main script and in the simulation state:
- Joint angles are defined in degrees.
- Joint velocities are defined in degrees per second.

Inside the dynamic model:
- Angles are converted to radians.
- Angular velocities are converted to radians per second.
- Accelerations are calculated in radians per second squared and then converted back to degrees per second squared.

This conversion is handled automatically by robotDynamics.m.

============================================================
8. Summary
============================================================

To use this simulation:

1. Open run_zero_input_simulation.m.
2. Change theta1_0, theta2_0, and theta3_0 if a different initial pose is required.
3. Keep in mind that these values are joint angles in degrees, not Cartesian coordinates.
4. Make sure all MATLAB files are in the same folder.
5. Run run_zero_input_simulation.m.
6. Review the generated plots and animation.

This simulation is intended to show the natural, uncontrolled motion of the robotic arm when the input torques are zero.
