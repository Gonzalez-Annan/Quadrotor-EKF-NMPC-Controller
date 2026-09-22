%% test_ekf_montecarlo.m
% test_ekf.m checks +-2sigma coverage from a single 30 s run. Errors in a
% Kalman filter are highly autocorrelated in time, so a single run's
% coverage percentage is a noisy estimate -- one unlucky patch of a few
% seconds can pull it well below any fixed threshold even for a filter
% that is behaving correctly on average. This script runs many
% independent trials and averages coverage across trials instead of
% across time, which is the statistically appropriate way to check
% whether the filter's reported uncertainty matches its actual error.
clear; clc;
params = getDefaultParams();
dt = 1/params.imu_rate_hz;

x_true = [0; 0; -2.5; zeros(3,1); 1; 0; 0; 0; zeros(3,1)];
u_hover = (params.m*params.g/4)*ones(4,1);
xdot_true = quadrotorDynamics(x_true, u_hover, params);

n_trials = 30;
T = 30;
N = round(T/dt);
labels = {'dp_x','dp_y','dp_z','dv_x','dv_y','dv_z','dth_x','dth_y','dth_z', ...
          'dba_x','dba_y','dba_z','dbg_x','dbg_y','dbg_z'};

coverage_per_trial = nan(n_trials, 15);
for trial = 1:n_trials
    rng(trial);
    e = ekf_init(x_true, params, true);
    ba_t = [0.03; -0.02; 0.01];
    bg_t = [0.002; -0.001; 0.001];
    err = nan(15,N); sig = nan(15,N);
    for k = 1:N
        [imu, ba_t, bg_t] = simulateIMU(x_true, xdot_true, ba_t, bg_t, dt, params);
        e = ekf_predict(e, imu.accel, imu.gyro, dt, params);
        if mod(k-1,5) == 0
            e = ekf_update(e, simulateGNSS(x_true, params), params);
        end
        err(:,k) = estimation_error(e, x_true, [ba_t; bg_t]);
        sig(:,k) = sqrt(max(0, diag(e.P)));
    end
    coverage_per_trial(trial,:) = mean(abs(err) <= 2*sig, 2)';
end

mean_coverage = mean(coverage_per_trial, 1);
std_coverage  = std(coverage_per_trial, 0, 1);

fprintf('--- EKF consistency, averaged over %d independent hover trials ---\n', n_trials);
for i = 1:15
    fprintf('  %-6s mean %5.1f%%  (trial-to-trial std %4.1f%%, worst trial %5.1f%%)\n', ...
        labels{i}, 100*mean_coverage(i), 100*std_coverage(i), 100*min(coverage_per_trial(:,i)));
end

figure('Name','EKF Monte Carlo Consistency', 'Position',[100 100 850 500]);
subplot(2,1,1);
bar(100*mean_coverage); hold on;
plot([0 16], [90 90], 'r--', 'LineWidth', 1.5);
set(gca, 'XTick', 1:15, 'XTickLabel', labels, 'XTickLabelRotation', 45);
ylabel('Mean coverage across trials [%]');
title(sprintf('Per-state \\pm2\\sigma Coverage, Averaged Over %d Trials', n_trials));
grid on;

subplot(2,1,2);
worst_state_per_trial = min(coverage_per_trial, [], 2);
plot(1:n_trials, 100*worst_state_per_trial, 'bo-'); hold on;
plot([1 n_trials], [90 90], 'r--', 'LineWidth', 1.5);
xlabel('Trial (random seed)'); ylabel('Min coverage that trial [%]');
title('This Is What test\_ekf.m''s Single-Seed Number Actually Looks Like Across Trials');
grid on;
saveas(gcf, 'test_ekf_montecarlo.png');

fprintf('\nA state genuinely undermodeled (covariance too small) will show\n');
fprintf('mean coverage well below 90%% here, consistently, across trials.\n');
fprintf('A state that is merely SLOW to observe (e.g. gyro bias at pure\n');
fprintf('hover, with no rotational excitation) will show high trial-to-\n');
fprintf('trial variance in the bottom plot even if its mean is reasonable --\n');
fprintf('that variance, not any single trial''s number, is the signature\n');
fprintf('of a weakly-observable state rather than a broken filter.\n');
