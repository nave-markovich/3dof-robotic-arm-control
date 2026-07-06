function gainInfo = printPotentialControllerGains(ctrl)

    % ============================================================
    % Print and return the attractive-potential controller gain data.
    %
    % This function should be called after running the main simulation
    % script, because it uses the controller structure ctrl that was
    % created there.
    %
    % Required fields:
    % ctrl.Katt
    % ctrl.Kd
    % ctrl.gainDesign.Ts
    % ctrl.gainDesign.zeta
    % ctrl.gainDesign.omega_n
    % ctrl.gainDesign.Mgoal
    % ctrl.gainDesign.Ieff
    % ctrl.gainDesign.thetaGoal_deg
    %
    % Output:
    % gainInfo is a structure that contains all printed values.
    % ============================================================

    requiredMainFields = {'Katt', 'Kd', 'gainDesign'};

    for i = 1:length(requiredMainFields)
        if ~isfield(ctrl, requiredMainFields{i})
            error('Missing required field ctrl.%s. Run the main simulation script first.', requiredMainFields{i});
        end
    end

    requiredDesignFields = {'Ts', 'zeta', 'omega_n', 'Mgoal', 'Ieff', 'thetaGoal_deg'};

    for i = 1:length(requiredDesignFields)
        if ~isfield(ctrl.gainDesign, requiredDesignFields{i})
            error(['Missing required field ctrl.gainDesign.%s.\n' ...
                   'Add the gainDesign storage block after Katt and Kd are computed in the main script.'], ...
                   requiredDesignFields{i});
        end
    end

    Ts = ctrl.gainDesign.Ts;
    zeta = ctrl.gainDesign.zeta;
    omega_n = ctrl.gainDesign.omega_n;
    Mgoal = ctrl.gainDesign.Mgoal;
    Ieff = ctrl.gainDesign.Ieff;
    thetaGoal_deg = ctrl.gainDesign.thetaGoal_deg;

    Katt = ctrl.Katt;
    Kd = ctrl.Kd;

    KattManualCheck = diag(Ieff .* omega_n.^2);
    KdManualCheck = diag(2 .* zeta .* Ieff .* omega_n);

    KattDifference = Katt - KattManualCheck;
    KdDifference = Kd - KdManualCheck;

    fprintf('\n============================================================\n');
    fprintf('Potential Controller Gain Report\n');
    fprintf('============================================================\n\n');

    fprintf('Target configuration used for gain design [deg]:\n');
    fprintf('thetaGoal_deg = [%.6f; %.6f; %.6f]\n\n', ...
        thetaGoal_deg(1), thetaGoal_deg(2), thetaGoal_deg(3));

    fprintf('Design formulas:\n');
    fprintf('omega_n = 4 ./ (zeta .* Ts)\n');
    fprintf('Katt = diag(Ieff .* omega_n.^2)\n');
    fprintf('Kd   = diag(2 .* zeta .* Ieff .* omega_n)\n\n');

    fprintf('Settling-time design vector Ts [s]:\n');
    fprintf('Ts = [%.6f; %.6f; %.6f]\n\n', Ts(1), Ts(2), Ts(3));

    fprintf('Damping-ratio vector zeta [-]:\n');
    fprintf('zeta = [%.6f; %.6f; %.6f]\n\n', zeta(1), zeta(2), zeta(3));

    fprintf('Natural-frequency vector omega_n [rad/s]:\n');
    fprintf('omega_n = [%.6f; %.6f; %.6f]\n\n', ...
        omega_n(1), omega_n(2), omega_n(3));

    fprintf('Mass matrix at the target configuration Mgoal:\n');
    disp(Mgoal);

    fprintf('Effective inertia vector Ieff = diag(Mgoal):\n');
    fprintf('Ieff = [%.6e; %.6e; %.6e]\n\n', ...
        Ieff(1), Ieff(2), Ieff(3));

    fprintf('Attractive stiffness gain matrix Katt [N*m/rad]:\n');
    disp(Katt);

    fprintf('Damping gain matrix Kd [N*m*s/rad]:\n');
    disp(Kd);

    fprintf('Manual verification of Katt calculation:\n');
    fprintf('max(abs(Katt - diag(Ieff .* omega_n.^2))) = %.6e\n\n', ...
        max(abs(KattDifference(:))));

    fprintf('Manual verification of Kd calculation:\n');
    fprintf('max(abs(Kd - diag(2 .* zeta .* Ieff .* omega_n))) = %.6e\n\n', ...
        max(abs(KdDifference(:))));

    fprintf('Per-joint scalar values:\n');
    fprintf('------------------------------------------------------------\n');
    fprintf('Joint     Ts [s]      zeta [-]    omega_n [rad/s]     Ieff [kg*m^2]       Katt          Kd\n');
    fprintf('------------------------------------------------------------\n');

    for i = 1:3
        fprintf('%d      %10.6f   %10.6f   %14.6f   %14.6e   %12.6e   %12.6e\n', ...
            i, Ts(i), zeta(i), omega_n(i), Ieff(i), Katt(i,i), Kd(i,i));
    end

    fprintf('------------------------------------------------------------\n\n');

    gainInfo.thetaGoal_deg = thetaGoal_deg;
    gainInfo.Ts = Ts;
    gainInfo.zeta = zeta;
    gainInfo.omega_n = omega_n;
    gainInfo.Mgoal = Mgoal;
    gainInfo.Ieff = Ieff;
    gainInfo.Katt = Katt;
    gainInfo.Kd = Kd;
    gainInfo.KattManualCheck = KattManualCheck;
    gainInfo.KdManualCheck = KdManualCheck;
    gainInfo.KattDifference = KattDifference;
    gainInfo.KdDifference = KdDifference;

end