function assertJointTargetInRange(theta_deg, jointLimits_deg, configurationName)

    % ============================================================
    % Check whether a joint-space configuration is inside the
    % allowed robot joint limits.
    % ============================================================

    if nargin < 3
        configurationName = 'configuration';
    end

    theta_deg = theta_deg(:);

    lowerLimits = jointLimits_deg(:, 1);
    upperLimits = jointLimits_deg(:, 2);

    belowLimit = theta_deg < lowerLimits;
    aboveLimit = theta_deg > upperLimits;

    if any(belowLimit | aboveLimit)

        invalidJoints = find(belowLimit | aboveLimit);

        message = sprintf('%s is outside the robot joint limits.\n', configurationName);

        for i = 1:length(invalidJoints)
            j = invalidJoints(i);
            message = sprintf('%sJoint %d: target = %.3f [deg], allowed range = [%.3f, %.3f] [deg]\n', ...
                message, j, theta_deg(j), lowerLimits(j), upperLimits(j));
        end

        error(message);

    end

end
