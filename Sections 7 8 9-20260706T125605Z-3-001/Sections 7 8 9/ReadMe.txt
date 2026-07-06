README.txt
3-DOF Robot Arm Motion Planning and Control Project

============================================================
ENGLISH
============================================================

1. Required software
- MATLAB.
- Put all project .m files in the same folder.
- Make sure this folder is the current MATLAB working directory.

2. Main files
- run_zero_input_simulation.m
  Runs the robot simulation with zero input torques.
  Use it for the zero-input motion graphs and stick-figure.

- run_potential_control_simulation.m
  Runs the controlled simulation using the attractive potential controller.
  The user enters a Cartesian target point [x y z] in meters.

3. How to run
In the MATLAB Command Window, run:

run_zero_input_simulation

or:

run_potential_control_simulation

4. Cartesian target input
When running the potential-control simulation, MATLAB asks for a target point.
Enter the point as a MATLAB vector, for example:

[0.25 0.45 0.20]

or:

[0.25, 0.45, 0.20]

Do not enter the point without square brackets.

5. Workspace range
The target is checked by radius from the workspace center:

workspace center = [0, l1, 0]

The allowed radius is:

abs(l3 - l2) <= r <= l2 + l3

For the current robot parameters:

minimum radius = 0.083 m
maximum radius = 0.605 m

The code prints this range before asking for the target.
If the target is outside the reachable range, MATLAB returns an error message.

6. Controller logic
The controller is based on an attractive potential function:

Uatt = 0.5*(q - qd)'*Katt*(q - qd)

The control torque is:

tau = G(q) - grad(Uatt) - Kd*dq

This means:
- G(q) compensates gravity.
- Katt pulls the robot toward the target.
- Kd damps the motion and reduces oscillations.

7. Gain parameters
Katt and Kd are computed from:
- Ts: desired settling time [s]
- zeta: damping ratio
- Mgoal: inertia matrix at the target configuration
- Ieff: diagonal terms of Mgoal
- omega_n = 4 ./ (zeta .* Ts)

A typical starting choice is:

Ts = [2.0; 2.0; 2.0]
zeta = [1.0; 1.0; 1.0]

If motion is too slow, reduce Ts.
If motion oscillates, increase zeta.
If torques are too large, increase Ts or enable torque saturation.

8. Torque saturation
The code may include:

ctrl.useTorqueSaturation = false;
ctrl.tauMax = [20; 20; 20];

If useTorqueSaturation is false, the torque is not limited.
To limit each joint torque to +/-20 N*m, set:

ctrl.useTorqueSaturation = true;

9. Output graphs
The code should generate:
- Joint angles vs. time: theta1, theta2, theta3
- Joint velocities vs. time
- Control torques vs. time
- Attractive potential energy
- End-effector Cartesian position vs. time
- Stick-figure visualization

The most important required motion graph is:
joint configuration parameters vs. time.

10. Animation note
If MATLAB does not release the Command Window, the animation is probably still running.
In animateRobotPotential.m, keep:

runAnimation = false;

to display only the final pose and finish immediately.
Set runAnimation = true only when you want to watch the full animation.

11. Optional gain report
If the function printPotentialControllerGains.m exists, run after the main simulation:

printPotentialControllerGains(ctrl)

This prints Katt, Kd, Ts, zeta, omega_n, Mgoal, and Ieff.

