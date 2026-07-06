function animateRobotPotential(t, x, p, thetaGoal_deg, targetPoint_m)

    % ============================================================
    % Stick-figure visualization for the controlled robot motion.
    % By default, the function shows the final robot configuration only
    % and returns immediately, so the Command Window is released.
    %
    % To watch the full animation, set runAnimation = true below.
    %
    % The origin [0,0,0] is kept at the center of the graph.
    % Fixed camera and fixed axes are used so the full motion is visible.
    % ============================================================

    % ------------------------------------------------------------
    % Animation control.
    % Keep runAnimation = false to release the Command Window immediately.
    % Set runAnimation = true only when you want to watch the animation.
    % ------------------------------------------------------------
    runAnimation = true;

    % ------------------------------------------------------------
    % Animation speed settings.
    % These values are used only if runAnimation = true.
    % Larger frameStep and playbackSpeed make the animation finish faster.
    % ------------------------------------------------------------
    frameStep = 5;
    playbackSpeed = 20.0;

    if nargin < 5 || isempty(targetPoint_m)
        targetPoints = robotPoints(thetaGoal_deg, p);
        targetPoint_m = targetPoints(:, end);
    else
        targetPoint_m = targetPoint_m(:);
    end

    n = length(t);

    % ------------------------------------------------------------
    % Precompute all robot points to find the full motion range.
    % ------------------------------------------------------------
    allX = [];
    allY = [];
    allZ = [];

    for k = 1:n
        q_deg = x(k, 1:3).';
        points = robotPoints(q_deg, p);

        allX = [allX, points(1,:)];
        allY = [allY, points(2,:)];
        allZ = [allZ, points(3,:)];
    end

    targetPoints = robotPoints(thetaGoal_deg, p);

    allX = [allX, targetPoints(1,:), targetPoint_m(1), 0];
    allY = [allY, targetPoints(2,:), targetPoint_m(2), 0];
    allZ = [allZ, targetPoints(3,:), targetPoint_m(3), 0];

    % Because the plotting convention is X-Z-Y:
    plotX = allX;
    plotY = allZ;
    plotZ = allY;

    margin = 0.08;

    % Symmetric limits keep [0,0,0] at the center of the graph.
    axisMax = max(abs([plotX, plotY, plotZ])) + margin;
    axisMax = max(axisMax, p.l1 + p.l2 + p.l3 + margin);

    xLimits = [-axisMax, axisMax];
    yLimits = [-axisMax, axisMax];
    zLimits = [-axisMax, axisMax];

    % ------------------------------------------------------------
    % Create figure.
    % ------------------------------------------------------------
    figure;
    hold on;
    grid on;
    axis equal;
    axis manual;

    xlabel('X [m]');
    ylabel('Z [m]');
    zlabel('Y [m]');
    title('Potential-control stick-figure robot motion');

    xlim(xLimits);
    ylim(yLimits);
    zlim(zLimits);

    view(45, 25);
    camproj perspective;

    % ------------------------------------------------------------
    % If animation is disabled, show only the final robot position
    % and return immediately.
    % ------------------------------------------------------------
    if ~runAnimation

        q_final_deg = x(end, 1:3).';
        finalPoints = robotPoints(q_final_deg, p);

        plot3(finalPoints(1,:), finalPoints(3,:), finalPoints(2,:), '-o', ...
            'LineWidth', 2.5, ...
            'MarkerSize', 7);

        plot3(0, 0, 0, 'ks', ...
            'MarkerSize', 8, ...
            'MarkerFaceColor', 'k');

        plot3(targetPoint_m(1), targetPoint_m(3), targetPoint_m(2), 'rp', ...
            'MarkerSize', 12, ...
            'MarkerFaceColor', 'r');

        plot3(xLimits, [0 0], [0 0], 'k:', 'LineWidth', 0.8);
        plot3([0 0], yLimits, [0 0], 'k:', 'LineWidth', 0.8);
        plot3([0 0], [0 0], zLimits, 'k:', 'LineWidth', 0.8);

        title(sprintf('Final robot configuration, t = %.2f s', t(end)));

        drawnow;

        fprintf('Stick-figure final pose displayed. Animation skipped, Command Window released.\n');

        return;

    end

    % ------------------------------------------------------------
    % Full animation mode.
    % This part runs only if runAnimation = true.
    % ------------------------------------------------------------
    q_deg = x(1, 1:3).';
    points = robotPoints(q_deg, p);

    hRobot = plot3(points(1,:), points(3,:), points(2,:), '-o', ...
        'LineWidth', 2.5, ...
        'MarkerSize', 7);

    plot3(0, 0, 0, 'ks', ...
        'MarkerSize', 8, ...
        'MarkerFaceColor', 'k');

    plot3(targetPoint_m(1), targetPoint_m(3), targetPoint_m(2), 'rp', ...
        'MarkerSize', 12, ...
        'MarkerFaceColor', 'r');

    % Draw reference axes through the origin.
    plot3(xLimits, [0 0], [0 0], 'k:', 'LineWidth', 0.8);
    plot3([0 0], yLimits, [0 0], 'k:', 'LineWidth', 0.8);
    plot3([0 0], [0 0], zLimits, 'k:', 'LineWidth', 0.8);

    for k = 1:frameStep:n

        q_deg = x(k, 1:3).';
        points = robotPoints(q_deg, p);

        set(hRobot, ...
            'XData', points(1,:), ...
            'YData', points(3,:), ...
            'ZData', points(2,:));

        title(sprintf('Potential-control stick figure, t = %.2f s', t(k)));

        drawnow;

        if k < n
            pause((t(min(k + frameStep, n)) - t(k)) / playbackSpeed);
        end

    end

    fprintf('Stick-figure animation finished. Command Window released.\n');

end