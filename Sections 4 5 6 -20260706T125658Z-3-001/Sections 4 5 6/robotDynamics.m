function xdot = robotDynamics(~, x, p)

    % State vector in degrees:
    % x = [theta1_deg; theta2_deg; theta3_deg;
    %      dtheta1_deg; dtheta2_deg; dtheta3_deg]

    q_deg = x(1:3);
    dq_deg = x(4:6);

    % Convert to radians for dynamics calculation
    q_rad = deg2rad(q_deg);
    dq_rad = deg2rad(dq_deg);

    % Zero input torques [N*m]
    tau = [0; 0; 0];

    % Dynamic model, calculated in radians
    M = inertiaMatrix(q_rad, p);
    C = coriolisMatrix(q_rad, dq_rad, p);
    G = gravityVector(q_rad, p);

    % Equations of motion:
    % M(q)*ddq + C(q,dq)*dq + G(q) = tau
    % ddq_rad is in [rad/s^2]
    ddq_rad = M \ (tau - C*dq_rad - G);

    % Convert acceleration back to [deg/s^2]
    ddq_deg = rad2deg(ddq_rad);

    % First-order state-space form, returned in degrees
    xdot = [
        dq_deg;
        ddq_deg
    ];

end