function [tau, Uatt, gradU] = potentialTorque(q_rad, dq_rad_s, G, ctrl)

    % ============================================================
    % Attractive potential controller.
    %
    % Uatt = 0.5*(q - qd)'*Katt*(q - qd)
    % gradU = Katt*(q - qd)
    %
    % With gravity compensation:
    % tau = G(q) - gradU - Kd*dq
    %
    % Without gravity compensation:
    % tau = -gradU - Kd*dq
    % ============================================================

    qd_rad = deg2rad(ctrl.thetaGoal_deg(:));

    e = q_rad - qd_rad;

    Uatt = 0.5 * e.' * ctrl.Katt * e;

    gradU = ctrl.Katt * e;

    if ctrl.useGravityCompensation
        tau = G - gradU - ctrl.Kd * dq_rad_s;
    else
        tau = -gradU - ctrl.Kd * dq_rad_s;
    end

    if isfield(ctrl, 'useTorqueSaturation') && ctrl.useTorqueSaturation
        tauMax = ctrl.tauMax(:);
        tau = min(max(tau, -tauMax), tauMax);
    end

end
