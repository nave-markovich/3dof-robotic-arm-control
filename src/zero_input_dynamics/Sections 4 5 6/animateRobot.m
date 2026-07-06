function animateRobot(t, x, p)

    % ============================================================
    % Smooth stick-figure animation
    % Fixed camera, fixed axes, full motion visible
    % No floor constraint
    % ============================================================

    n = length(t);

    % ------------------------------------------------------------
    % Precompute all robot points to find the full motion range
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

    % Because we plot as X-Z-Y:
    plotX = allX;
    plotY = allZ;
    plotZ = allY;

    minX = min(plotX); maxX = max(plotX);
    minY = min(plotY); maxY = max(plotY);
    minZ = min(plotZ); maxZ = max(plotZ);

    % Add a small margin, but keep the view close
    margin = 0.08;

    xLimits = [minX - margin, maxX + margin];
    yLimits = [minY - margin, maxY + margin];
    zLimits = [minZ - margin, maxZ + margin];

    % Make sure the base is visible
    xLimits = [min(xLimits(1), -0.05), max(xLimits(2), 0.05)];
    yLimits = [min(yLimits(1), -0.05), max(yLimits(2), 0.05)];
    zLimits = [min(zLimits(1), -0.05), max(zLimits(2), 0.05)];

    % ------------------------------------------------------------
    % Create figure
    % ------------------------------------------------------------
    figure;
    hold on;
    grid on;
    axis equal;
    axis manual;

    xlabel('X [m]');
    ylabel('Z [m]');
    zlabel('Y [m]');
    title('Zero-input stick-figure robot motion');

    xlim(xLimits);
    ylim(yLimits);
    zlim(zLimits);

    % Fixed camera
    view(45, 25);
    camproj perspective;

    % First robot pose
    q_deg = x(1, 1:3).';
    points = robotPoints(q_deg, p);

    hRobot = plot3(points(1,:), points(3,:), points(2,:), '-o', ...
        'LineWidth', 2.5, ...
        'MarkerSize', 7);

    % Draw fixed base point
    plot3(0, 0, 0, 'ks', ...
        'MarkerSize', 8, ...
        'MarkerFaceColor', 'k');

    % ------------------------------------------------------------
    % Smooth animation
    % ------------------------------------------------------------
    frameStep = 1;        % use every simulation point
    playbackSpeed = 1.0;  % 1 = real time, 2 = twice faster

    for k = 1:frameStep:n

        q_deg = x(k, 1:3).';
        points = robotPoints(q_deg, p);

        set(hRobot, ...
            'XData', points(1,:), ...
            'YData', points(3,:), ...
            'ZData', points(2,:));

        title(sprintf('Zero-input stick figure, t = %.2f s', t(k)));

        drawnow;

        if k < n
            pause((t(min(k+frameStep,n)) - t(k)) / playbackSpeed);
        end

    end

end