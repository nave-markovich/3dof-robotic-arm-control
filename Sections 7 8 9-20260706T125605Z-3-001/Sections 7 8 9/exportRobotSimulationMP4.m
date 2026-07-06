function exportRobotSimulationMP4(t, x, p, thetaGoal_deg, targetPoint_m, outputFileName)

    % ============================================================
    % Export the robot stick-figure simulation to an MP4 video file.
    %
    % Inputs:
    % t              - time vector [s]
    % x              - state matrix, columns 1:3 are joint angles [deg]
    % p              - robot parameter structure
    % thetaGoal_deg  - target joint configuration [deg]
    % targetPoint_m  - Cartesian target point [m]
    % outputFileName - output MP4 file name
    %
    % Example:
    % exportRobotSimulationMP4(t, x, p, thetaGoal_deg, targetPoint_m, 'robot_simulation.mp4');
    % ============================================================

    if nargin < 5 || isempty(targetPoint_m)
        targetPoints = robotPoints(thetaGoal_deg, p);
        targetPoint_m = targetPoints(:, end);
    else
        targetPoint_m = targetPoint_m(:);
    end

    if nargin < 6 || isempty(outputFileName)
        outputFileName = 'robot_simulation.mp4';
    end

    n = length(t);

    % ------------------------------------------------------------
    % Video settings.
    % Increase frameStep for shorter video files.
    % ------------------------------------------------------------
    frameStep = 3;
    videoFrameRate = 30;

    videoObj = VideoWriter(outputFileName, 'MPEG-4');
    videoObj.FrameRate = videoFrameRate;
    videoObj.Quality = 95;

    open(videoObj);

    % ------------------------------------------------------------
    % Precompute all robot points to set fixed symmetric axes.
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

    % Plotting convention: X-Z-Y
    plotX = allX;
    plotY = allZ;
    plotZ = allY;

    margin = 0.08;

    axisMax = max(abs([plotX, plotY, plotZ])) + margin;
    axisMax = max(axisMax, p.l1 + p.l2 + p.l3 + margin);

    xLimits = [-axisMax, axisMax];
    yLimits = [-axisMax, axisMax];
    zLimits = [-axisMax, axisMax];

    % ------------------------------------------------------------
    % Create figure for video frames.
    % ------------------------------------------------------------
    fig = figure('Color', 'w');
    hold on;
    grid on;
    axis equal;
    axis manual;

    xlabel('X [m]');
    ylabel('Z [m]');
    zlabel('Y [m]');

    xlim(xLimits);
    ylim(yLimits);
    zlim(zLimits);

    view(45, 25);
    camproj perspective;

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

    plot3(xLimits, [0 0], [0 0], 'k:', 'LineWidth', 0.8);
    plot3([0 0], yLimits, [0 0], 'k:', 'LineWidth', 0.8);
    plot3([0 0], [0 0], zLimits, 'k:', 'LineWidth', 0.8);

    % ------------------------------------------------------------
    % Write video frames.
    % No pause is used, so the export finishes as fast as MATLAB can run.
    % ------------------------------------------------------------
    for k = 1:frameStep:n

        q_deg = x(k, 1:3).';
        points = robotPoints(q_deg, p);

        set(hRobot, ...
            'XData', points(1,:), ...
            'YData', points(3,:), ...
            'ZData', points(2,:));

        title(sprintf('Potential-control robot simulation, t = %.2f s', t(k)));

        drawnow;

        frame = getframe(fig);
        writeVideo(videoObj, frame);

    end

    close(videoObj);

    fprintf('MP4 video exported successfully: %s\n', outputFileName);

end