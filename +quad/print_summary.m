function print_summary(result)
%PRINT_SUMMARY One clean, readable summary of a run -- not a raw struct
%   dump. disp(result.metrics) mixes scalars, vectors shown as
%   "[3x1 double]", and no units, which is unreadable at a glance. This
%   prints exactly the numbers that matter, in one consistent format.
    m = result.metrics;
    c = result.config;

    line = repmat('=', 1, 60);
    fprintf('\n%s\n', line);
    fprintf(' MA6224 Quadrotor EKF + NMPC -- Run Summary\n');
    fprintf('%s\n', line);
    fprintf(' Mission duration:        %.1f s (seed %d)\n', m.finalTime, m.seed);
    fprintf(' Position RMSE (truth):   %.2f cm   (max %.2f cm)\n', ...
        100*m.positionRMSE, 100*m.positionMax);
    fprintf(' Position RMSE (EKF est): %.2f cm\n', 100*m.estimatedPositionRMSE);
    fprintf(' Rotor thrust range:      [%.2f, %.2f] N   (limits: %.1f-%.1f N)\n', ...
        min(m.minRotorThrust), max(m.maxRotorThrust), c.thrustMin, c.thrustMax);
    fprintf(' Max roll / pitch:        %.1f deg / %.1f deg   (limit: +-%.0f deg)\n', ...
        m.maxAbsRollPitchDeg(1), m.maxAbsRollPitchDeg(2), c.tiltMax*180/pi);
    fprintf(' Max body rate:           [%.1f, %.1f, %.1f] deg/s   (limit: +-[%.0f,%.0f,%.0f] deg/s)\n', ...
        m.maxAbsBodyRatesDeg(1), m.maxAbsBodyRatesDeg(2), m.maxAbsBodyRatesDeg(3), ...
        c.rateMax(1)*180/pi, c.rateMax(2)*180/pi, c.rateMax(3)*180/pi);
    fprintf(' Constraint violations:   %d rotor, %d attitude, %d rate (samples)\n', ...
        m.rotorViolationSamples, m.attitudeViolationSamples, m.rateViolationSamples);
    fprintf(' NMPC solve time:         mean %.0f ms, max %.0f ms, p95 %.0f ms\n', ...
        1000*m.solveMean, 1000*m.solveMax, 1000*m.solveP95);
    fprintf(' Control period:          %.0f ms  ->  %d/%d updates exceeded it\n', ...
        1000*c.controlDt, m.deadlineMisses, m.controlUpdates);
    fprintf(' GNSS updates rejected:   %d\n', m.gnssRejected);
    fprintf(' EKF fallback count:      %d\n', m.fallbackCount);
    fprintf('%s\n', line);
    fprintf(' required/  01_trajectory, 02_tracking_errors, 03_ekf_bounds,\n');
    fprintf('            04_actuators_constraints, flight_visualization.avi\n');
    fprintf(' extra/     05_solver_performance, metrics.json,\n');
    fprintf('            configuration.json, telemetry.csv\n');
    fprintf(' Folder:    %s\n', result.outputDir);
    fprintf('%s\n', line);
    if result.config.makeVideo
        fprintf('\n');
        fprintf('****************************************************************\n');
        fprintf('***   THE VIDEO DOES NOT OPEN AUTOMATICALLY.                 ***\n');
        fprintf('***   >>> PLEASE LOOK INSIDE THE RESULTS FOLDER FOR IT <<<   ***\n');
        fprintf('****************************************************************\n');
        fprintf('%s\n\n', fullfile(result.outputDir,'required','flight_visualization.avi'));
    end
end
