function [imu, ba_next, bg_next] = simulateIMU(x_true, xdot_true, ba, bg, dt, params)
%SIMULATEIMU Simulated accelerometer + gyroscope measurement (Section 4).
%   Plays the role of IMU HARDWARE: takes the true (noise-free) state and
%   its true rate of change, and returns what a real, imperfect sensor
%   would report. This function must NEVER be called by ekf.m with
%   anything derived from ekf.m's own estimate -- only main_simulation.m
%   should ever see x_true/xdot_true, and it exists only to produce the
%   noisy imu struct that gets handed onward.
%
%   [imu, ba_next, bg_next] = simulateIMU(x_true, xdot_true, ba, bg, dt, params)
%
%   x_true    : 13x1 true state (see quadrotorDynamics.m for layout)
%   xdot_true : 13x1 true derivative, i.e. quadrotorDynamics(x_true,u,params)
%               evaluated with the ACTUAL thrust applied this step
%   ba, bg    : 3x1 current accelerometer / gyroscope bias (carry these
%               between calls -- bias is a random walk, not iid noise,
%               so it has memory; see Test 3 in test_sensors.m)
%   dt        : time since the previous IMU sample [s] (1/params.imu_rate_hz)
%   params    : struct from getDefaultParams()
%
%   Returns:
%   imu       : struct with fields .accel (3x1, body-frame specific
%               force, m/s^2) and .gyro (3x1, body rate, rad/s)
%   ba_next, bg_next : updated bias, to be fed into the NEXT call

    q     = x_true(7:10);
    q     = q / norm(q);
    omega_true = x_true(11:13);
    accel_world_true = xdot_true(4:6);

    % An accelerometer does not sense gravity (it measures SPECIFIC
    % FORCE -- the non-gravitational part of acceleration, which is why
    % an accelerometer at rest on a table reads ~9.81, not 0). Subtract
    % gravity in world frame, then rotate into the body frame the
    % physical sensor is mounted in.
    R = quat2rotmCustom(q);
    g_world = [0; 0; params.g];
    specific_force_body_true = R' * (accel_world_true - g_world);

    % Bias random walk: b_k = b_{k-1} + w, w ~ N(0, sigma_rw^2 * dt).
    % The sqrt(dt) scaling is what makes this a random WALK rather than
    % iid noise -- accumulated variance grows linearly with elapsed
    % time, so standard deviation grows with sqrt(time). See Test 3.
    ba_next = ba + params.sigma_ba_rw * sqrt(dt) * randn(3,1);
    bg_next = bg + params.sigma_bg_rw * sqrt(dt) * randn(3,1);

    imu.accel = specific_force_body_true + ba_next + params.sigma_accel * randn(3,1);
    imu.gyro  = omega_true               + bg_next + params.sigma_gyro  * randn(3,1);
end