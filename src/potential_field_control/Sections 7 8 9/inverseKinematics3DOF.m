function [theta_deg, info] = inverseKinematics3DOF(target_m, p, jointLimits_deg, referenceTheta_deg)

    % ============================================================
    % Inverse kinematics for the current 3-DOF robot geometry.
    %
    % Input:
    % target_m = [x; y; z] end-effector target point in meters.
    % p contains l1, l2, l3.
    % jointLimits_deg is a 3x2 matrix in degrees.
    % referenceTheta_deg is used to choose the closest valid IK branch.
    %
    % Output:
    % theta_deg = [theta1; theta2; theta3] in degrees.
    % info contains all IK candidates and the selected branch index.
    % ============================================================

    if nargin < 4 || isempty(referenceTheta_deg)
        referenceTheta_deg = [0; 0; 0];
    end

    target_m = target_m(:);
    referenceTheta_deg = referenceTheta_deg(:);

    if numel(target_m) ~= 3
        error('Cartesian target must be a 3-element vector: [x; y; z].');
    end

    if any(~isfinite(target_m))
        error('Cartesian target contains non-finite values.');
    end

    x = target_m(1);
    y = target_m(2);
    z = target_m(3);

    r = hypot(x, z);
    s = y - p.l1;
    d = hypot(r, s);

    minReach = abs(p.l2 - p.l3);
    maxReach = p.l2 + p.l3;
    reachTolerance = 1e-10;

    if d > maxReach + reachTolerance || d < minReach - reachTolerance
        error(['Cartesian target is outside the reachable workspace.\n' ...
               'Target = [%.4f, %.4f, %.4f] [m].\n' ...
               'Distance from shoulder plane = %.4f [m].\n' ...
               'Allowed distance range = [%.4f, %.4f] [m].'], ...
               x, y, z, d, minReach, maxReach);
    end

    D = (r^2 + s^2 - p.l2^2 - p.l3^2) / (2*p.l2*p.l3);
    D = min(max(D, -1), 1);

    if r < 1e-12
        theta1 = deg2rad(referenceTheta_deg(1));
    else
        theta1 = atan2(z, x);
    end

    theta3Candidates = [
        atan2( sqrt(max(0, 1 - D^2)), D);
        atan2(-sqrt(max(0, 1 - D^2)), D)
    ];

    candidates_deg = zeros(3, 2);

    for i = 1:2
        theta3 = theta3Candidates(i);
        theta2 = atan2(s, r) - atan2(p.l3*sin(theta3), p.l2 + p.l3*cos(theta3));

        candidates_deg(:, i) = rad2deg([theta1; theta2; theta3]);
        candidates_deg(:, i) = normalizeAnglesDeg(candidates_deg(:, i));
    end

    lowerLimits = jointLimits_deg(:, 1);
    upperLimits = jointLimits_deg(:, 2);

    valid = false(1, 2);
    cost = inf(1, 2);

    for i = 1:2
        candidate = candidates_deg(:, i);
        valid(i) = all(candidate >= lowerLimits) && all(candidate <= upperLimits);

        if valid(i)
            diff_deg = normalizeAnglesDeg(candidate - referenceTheta_deg);
            cost(i) = norm(diff_deg);
        end
    end

    if ~any(valid)
        message = sprintf(['Cartesian target is geometrically reachable, but no IK branch satisfies the joint limits.\n' ...
                           'Target = [%.4f, %.4f, %.4f] [m].\n'], x, y, z);

        for i = 1:2
            message = sprintf('%sCandidate %d: theta = [%.3f, %.3f, %.3f] [deg]\n', ...
                message, i, candidates_deg(1,i), candidates_deg(2,i), candidates_deg(3,i));
        end

        error(message);
    end

    [~, selectedIndex] = min(cost);
    theta_deg = candidates_deg(:, selectedIndex);

    info.target_m = target_m;
    info.r = r;
    info.s = s;
    info.distanceFromShoulder = d;
    info.minReach = minReach;
    info.maxReach = maxReach;
    info.candidates_deg = candidates_deg;
    info.validCandidates = valid;
    info.selectedIndex = selectedIndex;

end

function angle_deg = normalizeAnglesDeg(angle_deg)

    % ============================================================
    % Normalize angles to the interval [-180, 180] degrees.
    % ============================================================

    angle_deg = mod(angle_deg + 180, 360) - 180;

end
