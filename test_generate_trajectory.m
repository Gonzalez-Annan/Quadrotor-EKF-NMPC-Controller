%% test_generateTrajectory.m
% Visual + numeric validation of generateTrajectory.m, run BEFORE this
% reference is trusted by the NMPC. Four checks:
%   1. Shape          -- does it actually look like a 3D figure-8?
%   2. Period         -- does it repeat every 2*pi/w0 seconds, as designed?
%   3. Derivatives    -- do the analytic vel/acc match finite differences
%                        of the position/velocity signals?
%   4. Yaw continuity -- the +/-180 deg wraparound, shown raw vs. fixed
clear; clc; close all;

params = getDefaultParams();
t = linspace(0, params.traj.tf, 3000); % dense sampling for smooth plots
traj = generateTrajectory(t, params);

%% TEST 1 -- Shape
fprintf('--- Test 1: Trajectory shape ---\n');
figure('Name','Test 1: Trajectory Shape', 'Position',[100 100 900 700]);

subplot(2,2,[1 3]);
plot3(traj.pos(1,:), traj.pos(2,:), traj.pos(3,:), 'b-', 'LineWidth', 1.5);
grid on; axis equal; view(45,25);
xlabel('x, North [m]'); ylabel('y, East [m]'); zlabel('z, Down [m]');
title('Test 1a: Full 3D Path (expected lemniscate / figure-8)');
set(gca, 'ZDir', 'reverse'); % NED: plot with "up" visually up, even though z is down-positive

subplot(2,2,2);
plot(traj.pos(1,:), traj.pos(2,:), 'b-', 'LineWidth', 1.5);
grid on; axis equal;
xlabel('x, North [m]'); ylabel('y, East [m]');
title('Test 1b: Top-Down View (figure-8 should be obvious here)');

subplot(2,2,4);
altitude = -traj.pos(3,:); % NED->intuitive: flip sign, "up" is now positive
plot(traj.t, altitude, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]'); ylabel('Altitude, up = + [m]');
title('Test 1c: Altitude vs Time (sign-flipped from NED for readability)');
yline_val = -params.traj.z0;
hold on; plot(traj.t, yline_val*ones(size(traj.t)), 'k:');
legend('Altitude', sprintf('Nominal %.1f m', yline_val), 'Location','best');

saveas(gcf, 'test1_trajectory_shape.png');
fprintf('Inspect test1_trajectory_shape.png: 1a/1b should clearly show a\n');
fprintf('figure-8, and 1c should oscillate around %.1f m altitude with\n', -params.traj.z0);
fprintf('+/-%.1f m swing, never crossing zero (never touching the ground).\n', params.traj.Az);

%% TEST 2 -- Period
fprintf('\n--- Test 2: Period check ---\n');
T_period = 2*pi/params.traj.w0;
fprintf('Expected period: 2*pi/w0 = %.4f s\n', T_period);

t_check = [0, T_period];
traj_check = generateTrajectory(t_check, params);
pos_err = max(abs(traj_check.pos(:,1) - traj_check.pos(:,2)));

fprintf('Position at t=0 vs t=T_period, max component difference: %.3e m\n', pos_err);
if pos_err < 1e-6
    fprintf('PASS: trajectory repeats after one period, as designed\n');
else
    fprintf('FAIL: trajectory does not repeat -- check w0 usage in x/y/z equations\n');
end

%% TEST 3 -- Derivatives vs. finite differences
fprintf('\n--- Test 3: Analytic derivatives vs finite differences ---\n');
dt = t(2) - t(1);

vel_fd = diff(traj.pos, 1, 2) / dt;              % finite-difference velocity
vel_analytic_mid = (traj.vel(:,1:end-1) + traj.vel(:,2:end)) / 2; % analytic, same sample points as vel_fd
vel_err = max(abs(vel_fd - vel_analytic_mid), [], 2);

acc_fd = diff(traj.vel, 1, 2) / dt;
acc_analytic_mid = (traj.acc(:,1:end-1) + traj.acc(:,2:end)) / 2;
acc_err = max(abs(acc_fd - acc_analytic_mid), [], 2);

fprintf('Max velocity error [x y z]:     [%.3e %.3e %.3e] m/s\n', vel_err(1), vel_err(2), vel_err(3));
fprintf('Max acceleration error [x y z]: [%.3e %.3e %.3e] m/s^2\n', acc_err(1), acc_err(2), acc_err(3));

figure('Name','Test 3: Derivative Validation', 'Position',[100 100 800 600]);
subplot(2,1,1);
plot(t(1:end-1), vel_fd(1,:), 'b-', 'LineWidth', 2); hold on;
plot(t, traj.vel(1,:), 'r--', 'LineWidth', 1.2);
grid on; xlabel('Time [s]'); ylabel('x velocity [m/s]');
legend('Finite difference of position', 'Analytic formula', 'Location','best');
title('Test 3a: x-velocity -- Finite Diff vs Analytic (should overlap)');

subplot(2,1,2);
plot(t(1:end-1), acc_fd(1,:), 'b-', 'LineWidth', 2); hold on;
plot(t, traj.acc(1,:), 'r--', 'LineWidth', 1.2);
grid on; xlabel('Time [s]'); ylabel('x acceleration [m/s^2]');
legend('Finite difference of velocity', 'Analytic formula', 'Location','best');
title('Test 3b: x-acceleration -- Finite Diff vs Analytic (should overlap)');
saveas(gcf, 'test3_derivative_validation.png');

tol_vel = 0.05; % loose: finite-difference of a fast-varying signal has real O(dt) error
tol_acc = 0.5;
if all(vel_err < tol_vel) && all(acc_err < tol_acc)
    fprintf('PASS: analytic derivatives match finite differences\n');
else
    fprintf('FAIL: analytic derivative does not match finite-difference check\n');
end

%% TEST 4 -- Yaw continuity (the professor''s discontinuity comment)
fprintf('\n--- Test 4: Yaw continuity (raw vs unwrapped) ---\n');
psi_raw_deg = rad2deg(traj.psi_raw);
psi_deg     = rad2deg(traj.psi);

jumps_raw = sum(abs(diff(psi_raw_deg)) > 180);
jumps_fixed = sum(abs(diff(psi_deg)) > 180);
fprintf('Discontinuities (>180 deg jump between samples) in raw psi:      %d\n', jumps_raw);
fprintf('Discontinuities (>180 deg jump between samples) in unwrapped psi: %d\n', jumps_fixed);

figure('Name','Test 4: Yaw Continuity', 'Position',[100 100 800 600]);
subplot(2,1,1);
plot(t, psi_raw_deg, 'r-', 'LineWidth', 1.2);
grid on; xlabel('Time [s]'); ylabel('Raw \psi_r [deg]');
title(sprintf('Test 4a: Raw atan2 Yaw : %d Discontinuities', jumps_raw));
ylim([-200 200]);

subplot(2,1,2);
plot(t, psi_deg, 'b-', 'LineWidth', 1.2);
grid on; xlabel('Time [s]'); ylabel('Unwrapped \psi_r [deg]');
title(sprintf('Test 4b: unwrap()-ed Yaw : %d Discontinuities', jumps_fixed));
saveas(gcf, 'test4_yaw_continuity.png');

if jumps_raw > 0 && jumps_fixed == 0
    fprintf('PASS: unwrap() removes the artificial jumps; raw signal confirmed to have them\n');
else
    fprintf('FAIL: expected raw psi to jump and unwrapped psi to be smooth -- check unwrap() usage\n');
end