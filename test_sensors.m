%% test_sensors.m
% Visual + statistical validation of simulateIMU.m / simulateGNSS.m, run
% BEFORE the EKF is trusted to fuse their output. Four checks:
%   1. IMU noise statistics   -- do accel/gyro noise match Section 4's sigmas?
%   2. GNSS noise statistics  -- does position noise match sigma_pos?
%   3. Bias random walk       -- does bias drift grow like sqrt(time), as designed?
%   4. Sampling cadence       -- do IMU (100 Hz) and GNSS (20 Hz) interleave 5:1?
clear; clc; close all;
rng(42); % fixed seed: makes this test's pass/fail reproducible, not a
         % coin flip that occasionally fails on an unlucky random draw

params = getDefaultParams();

%% TEST 1 -- IMU noise statistics
fprintf('--- Test 1: IMU noise statistics ---\n');
x0 = [0;0;0; 0;0;0; 1;0;0;0; 0;0;0]; % hover, level, at rest (same as dynamics Test 1)
u_hover = (params.m*params.g/4) * ones(4,1);
xdot0 = quadrotorDynamics(x0, u_hover, params); % at hover, accel_world = 0 (see dynamics Test 1)

N = 20000;
accel_samples = zeros(3, N);
gyro_samples  = zeros(3, N);
for i = 1:N
    % dt = 0 deliberately zeroes the bias random-walk term (sqrt(0)=0),
    % isolating pure measurement noise -- bias behavior is Test 3's job.
    imu = simulateIMU(x0, xdot0, zeros(3,1), zeros(3,1), 0, params);
    accel_samples(:,i) = imu.accel;
    gyro_samples(:,i)  = imu.gyro;
end

% Expected: hover -> zero world acceleration -> specific force is purely
% -gravity in the body frame, which at level attitude is [0;0;-g].
accel_expected = [0; 0; -params.g];
gyro_expected  = [0; 0; 0];

accel_mean_err = max(abs(mean(accel_samples,2) - accel_expected));
gyro_mean_err  = max(abs(mean(gyro_samples,2)  - gyro_expected));
accel_std_err  = max(abs(std(accel_samples,0,2) - params.sigma_accel)) / params.sigma_accel;
gyro_std_err   = max(abs(std(gyro_samples,0,2)  - params.sigma_gyro))  / params.sigma_gyro;

fprintf('Accel mean error (should be ~0): %.4g m/s^2\n', accel_mean_err);
fprintf('Gyro mean error (should be ~0):  %.4g rad/s\n', gyro_mean_err);
fprintf('Accel std relative error: %.2f%% (sample vs sigma_accel=%.3f)\n', accel_std_err*100, params.sigma_accel);
fprintf('Gyro std relative error:  %.2f%% (sample vs sigma_gyro=%.3f)\n', gyro_std_err*100, params.sigma_gyro);

tol_mean_accel = 4*params.sigma_accel/sqrt(N); % ~4 standard errors of the mean
tol_mean_gyro  = 4*params.sigma_gyro/sqrt(N);
if accel_mean_err < tol_mean_accel && gyro_mean_err < tol_mean_gyro && ...
   accel_std_err < 0.05 && gyro_std_err < 0.05
    fprintf('PASS: IMU noise matches Section 4 sigmas\n');
else
    fprintf('FAIL: IMU noise statistics do not match expected sigmas\n');
end

figure('Name','Test 1: IMU Noise Distribution', 'Position',[100 100 800 350]);
subplot(1,2,1);
[counts, centers] = hist(accel_samples(3,:), 60);
binwidth = centers(2) - centers(1);
bar(centers, counts/(sum(counts)*binwidth), 1, 'FaceColor',[0.7 0.85 1]); hold on;
xs = linspace(accel_expected(3)-4*params.sigma_accel, accel_expected(3)+4*params.sigma_accel, 200);
plot(xs, normpdfCustom(xs, accel_expected(3), params.sigma_accel), 'r-', 'LineWidth',2);
xlabel('accel_z [m/s^2]'); ylabel('Probability density');
legend('Simulated samples', 'Theoretical N(-g, \sigma_a^2)', 'Location','best');
title('Test 1a: Accelerometer z-axis Noise');

subplot(1,2,2);
[counts, centers] = hist(gyro_samples(1,:), 60);
binwidth = centers(2) - centers(1);
bar(centers, counts/(sum(counts)*binwidth), 1, 'FaceColor',[0.7 0.85 1]); hold on;
xs = linspace(-4*params.sigma_gyro, 4*params.sigma_gyro, 200);
plot(xs, normpdfCustom(xs, 0, params.sigma_gyro), 'r-', 'LineWidth',2);
xlabel('gyro_x [rad/s]'); ylabel('Probability density');
legend('Simulated samples', 'Theoretical N(0, \sigma_g^2)', 'Location','best');
title('Test 1b: Gyroscope x-axis Noise');
saveas(gcf, 'test1_imu_noise.png');

%% TEST 2 -- GNSS noise statistics
fprintf('\n--- Test 2: GNSS noise statistics ---\n');
N_gnss = 20000;
pos_samples = zeros(3, N_gnss);
for i = 1:N_gnss
    pos_samples(:,i) = simulateGNSS(x0, params);
end
pos_mean_err = max(abs(mean(pos_samples,2) - x0(1:3)));
pos_std_err  = max(abs(std(pos_samples,0,2) - params.sigma_pos)) / params.sigma_pos;

fprintf('Position mean error (should be ~0): %.4g m\n', pos_mean_err);
fprintf('Position std relative error: %.2f%% (sample vs sigma_pos=%.3f)\n', pos_std_err*100, params.sigma_pos);

figure('Name','Test 2: GNSS Noise Distribution', 'Position',[100 100 450 350]);
[counts, centers] = hist(pos_samples(1,:), 60);
binwidth = centers(2) - centers(1);
bar(centers, counts/(sum(counts)*binwidth), 1, 'FaceColor',[0.7 0.85 1]); hold on;
xs = linspace(-4*params.sigma_pos, 4*params.sigma_pos, 200);
plot(xs, normpdfCustom(xs, 0, params.sigma_pos), 'r-', 'LineWidth',2);
xlabel('x position error [m]'); ylabel('Probability density');
legend('Simulated samples', 'Theoretical N(0, \sigma_p^2)', 'Location','best');
title('Test 2: GNSS x-axis Noise');
saveas(gcf, 'test2_gnss_noise.png');

tol_mean_pos = 4*params.sigma_pos/sqrt(N_gnss);
if pos_mean_err < tol_mean_pos && pos_std_err < 0.05
    fprintf('PASS: GNSS noise matches Section 4 sigma_pos\n');
else
    fprintf('FAIL: GNSS noise statistics do not match expected sigma\n');
end

%% TEST 3 -- Bias random walk grows like sqrt(time)
fprintf('\n--- Test 3: Bias random walk growth ---\n');
dt_imu = 1/params.imu_rate_hz;
T_mission = params.traj.tf;
n_steps = round(T_mission/dt_imu);
n_realizations = 300;

% Vectorized: each row is one independent realization's bias history.
% cumsum along time turns iid steps into a random WALK (each step adds
% onto the last), which is what makes variance grow with elapsed time.
steps = params.sigma_ba_rw * sqrt(dt_imu) * randn(n_realizations, n_steps);
ba_x_history = cumsum(steps, 2); % n_realizations x n_steps
t_hist = (1:n_steps) * dt_imu;

empirical_std_final = std(ba_x_history(:,end));
theoretical_std_final = params.sigma_ba_rw * sqrt(T_mission);
rel_err = abs(empirical_std_final - theoretical_std_final) / theoretical_std_final;

fprintf('Empirical std of bias at t=%.0fs (across %d runs): %.4g\n', T_mission, n_realizations, empirical_std_final);
fprintf('Theoretical std (sigma_ba_rw * sqrt(T)):            %.4g\n', theoretical_std_final);
fprintf('Relative error: %.1f%%\n', rel_err*100);

figure('Name','Test 3: Bias Random Walk', 'Position',[100 100 700 450]);
plot(t_hist, ba_x_history(1:20,:), 'Color',[0.65 0.75 0.95], 'LineWidth', 0.8); hold on;
envelope = params.sigma_ba_rw * sqrt(t_hist);
plot(t_hist, 2*envelope, 'k--', 'LineWidth', 1.5);
plot(t_hist, -2*envelope, 'k--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Accel bias, x-axis [m/s^2]');
title('Test 3: 20 Sample Bias Paths vs. Theoretical \pm2\sigma Envelope (grows as \surdt)');
legend('Sample realizations', '', '\pm2\sigma_{rw}\surdt envelope', 'Location','best');
grid on;
saveas(gcf, 'test3_bias_random_walk.png');

if rel_err < 0.15 % Monte Carlo with 300 runs: ~15% is a reasonable band
    fprintf('PASS: bias drift matches sqrt(time) growth\n');
else
    fprintf('FAIL: bias growth does not match theoretical random-walk scaling\n');
end

%% TEST 4 -- Sampling cadence: IMU (100 Hz) vs GNSS (20 Hz)
fprintf('\n--- Test 4: Sampling cadence ---\n');
t_window = 0.3; % short window, just to see the pattern clearly
t_imu_ticks  = 0:dt_imu:t_window;
dt_gnss = 1/params.gnss_rate_hz;
t_gnss_ticks = 0:dt_gnss:t_window;

fprintf('IMU ticks in %.1fs window:  %d (every %.3fs)\n', t_window, numel(t_imu_ticks), dt_imu);
fprintf('GNSS ticks in %.1fs window: %d (every %.3fs)\n', t_window, numel(t_gnss_ticks), dt_gnss);

% Direct check: every 5th IMU tick should land on a GNSS tick (within
% floating-point tolerance), since 100/20 = 5 exactly. A raw count ratio
% over a short window is a noisier way to check the same thing -- it
% picks up an edge effect from counting both endpoints of a small
% interval, which looks like a discrepancy but isn't one.
imu_per_gnss = params.imu_rate_hz / params.gnss_rate_hz;
aligned = true;
for k = 1:imu_per_gnss:numel(t_imu_ticks)
    if ~any(abs(t_imu_ticks(k) - t_gnss_ticks) < 1e-9)
        aligned = false;
        break;
    end
end
if aligned
    fprintf('PASS: every %dth IMU tick lines up exactly with a GNSS tick\n', imu_per_gnss);
else
    fprintf('FAIL: IMU/GNSS ticks do not align -- check imu_rate_hz/gnss_rate_hz\n');
end

figure('Name','Test 4: Sampling Cadence', 'Position',[100 100 700 250]);
plot(t_imu_ticks, ones(size(t_imu_ticks)), 'bx', 'MarkerSize', 8, 'LineWidth', 1.5); hold on;
plot(t_gnss_ticks, zeros(size(t_gnss_ticks)), 'rx', 'MarkerSize', 8, 'LineWidth', 1.5);
for k = 1:numel(t_imu_ticks)
    plot([t_imu_ticks(k) t_imu_ticks(k)], [0.9 1.1], 'b-', 'LineWidth', 0.5);
end
for k = 1:numel(t_gnss_ticks)
    plot([t_gnss_ticks(k) t_gnss_ticks(k)], [-0.1 0.1], 'r-', 'LineWidth', 0.5);
end
ylim([-0.5 1.5]);
set(gca, 'YTick', [0 1], 'YTickLabel', {'GNSS (20 Hz)','IMU (100 Hz)'});
xlabel('Time [s]'); grid on;
title('Test 4: Every 5th IMU Sample Should Line Up With a GNSS Sample');
saveas(gcf, 'test4_sampling_cadence.png');

fprintf('Inspect test4_sampling_cadence.png: every 5th blue tick should\n');
fprintf('land exactly on a red tick -- that alignment is what lets\n');
fprintf('main_simulation.m run the EKF predict step every IMU tick and\n');
fprintf('the correct step only on ticks shared with GNSS.\n');