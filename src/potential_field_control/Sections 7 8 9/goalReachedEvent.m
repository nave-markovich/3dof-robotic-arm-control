function [value, isterminal, direction] = goalReachedEvent(~, x, ctrl)

    % ============================================================
    % Stop integration when both the joint error and the joint
    % velocity are below the required tolerances.
    % ============================================================

    q_deg = x(1:3);
    dq_deg_s = x(4:6);

    positionRatio = norm(q_deg - ctrl.thetaGoal_deg(:)) / ctrl.goalTolerance_deg;
    velocityRatio = norm(dq_deg_s) / ctrl.velocityTolerance_deg_s;

    value = max(positionRatio, velocityRatio) - 1;

    isterminal = 1;
    direction = -1;

end
