function [xdot, tau, Uatt] = robotDynamicsPotential(~, x, p, ctrl)

    % ============================================================
    % Closed-loop robot dynamics using attractive potential control.
    %
    % State vector in degrees:
    % x = [theta1_deg; theta2_deg; theta3_deg;
    %      dtheta1_deg_s; dtheta2_deg_s; dtheta3_deg_s]
    %
    % Dynamics are calculated in radians:
    % M(q)*ddq + C(q,dq)*dq + G(q) = tau
    % ============================================================

    q_deg = x(1:3);
    dq_deg_s = x(4:6);

    % Convert to radians for dynamic calculations.
    q_rad = deg2rad(q_deg);
    dq_rad_s = deg2rad(dq_deg_s);

    % Dynamic model.
    M = inertiaMatrix(q_rad, p);
    C = coriolisMatrix(q_rad, dq_rad_s, p);
    G = gravityVector(q_rad, p);

    % Attractive potential control torque.
    [tau, Uatt] = potentialTorque(q_rad, dq_rad_s, G, ctrl);

    % Equations of motion:
    % M(q)*ddq + C(q,dq)*dq + G(q) = tau
    ddq_rad_s2 = M \ (tau - C*dq_rad_s - G);

    % Convert acceleration back to degrees.
    ddq_deg_s2 = rad2deg(ddq_rad_s2);

    % First-order state-space form.
    xdot = [
        dq_deg_s;
        ddq_deg_s2
    ];

end
