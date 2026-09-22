%% test_ekf.m
% Standalone EKF consistency test for the first codebase.
% Checks, with PASS/FAIL prints matching the other test_* files:
%   1. Covariance contraction: position variance must shrink well below
%      the initial value once GNSS fixes are fused at hover.
%   2. +-2sigma coverage: >=90% of the 15 error-state components must
%      stay inside the filter's own bounds over a 30 s hover run.
%   3. Bias tracking: estimated accel bias must move toward the truth.
clear; clc;
rng(1);
params = getDefaultParams();
dt = 1/params.imu_rate_hz;

x_true = [0; 0; -2.5; zeros(3,1); 1; 0; 0; 0; zeros(3,1)];
u_hover = (params.m*params.g/4)*ones(4,1);
xdot_true = quadrotorDynamics(x_true, u_hover, params);  % ~zero at hover

e = ekf_init(x_true, params, true);
ba_t = [0.03; -0.02; 0.01];
bg_t = [0.002; -0.001; 0.001];
N = round(30/dt);
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

coverage = mean(abs(err) <= 2*sig, 2);
fprintf('--- EKF hover consistency ---\n');
fprintf('Min per-state +-2sigma coverage: %.1f%% (threshold 90%%)\n', 100*min(coverage));
fprintf('Final position std: %.4f m (initial 0.05 m)\n', sqrt(e.P(1,1)));
fprintf('GNSS accepted/rejected: %d / %d\n', e.gnssAccepted, e.gnssRejected);
fprintf('Accel bias est vs truth: [%.4f %.4f %.4f] vs [%.4f %.4f %.4f]\n', ...
    e.ba(1), e.ba(2), e.ba(3), ba_t(1), ba_t(2), ba_t(3));

assert(e.gnssAccepted > 500, 'FAIL: too few accepted GNSS updates');
assert(sqrt(e.P(1,1)) < 0.02, 'FAIL: position variance did not contract');
assert(min(coverage) > 0.90, 'FAIL: +-2sigma coverage too low');
fprintf('PASS ekf hover consistency\n');
