%% test_quadrotorDynamics.m
% Visual + numeric validation of quadrotorDynamics.m, run BEFORE this
% model is trusted inside the EKF or NMPC. Three checks, each with an
% obvious pass/fail signal:
%   1. Hover equilibrium      -- numeric assert
%   2. Free fall              -- simulated vs. closed-form, plotted
%   3. Pure roll torque       -- roll/pitch/yaw overlay + position drift
clear; clc; close all;

params = getDefaultParams();

%% TEST 1 -- Hover equilibrium
fprintf('--- Test 1: Hover equilibrium ---\n');
x_hover = [0;0;0; 0;0;0; 1;0;0;0; 0;0;0]; % level attitude, at rest
T_hover = params.m * params.g / 4;
u_hover = T_hover * ones(4,1);

xdot = quadrotorDynamics(x_hover, u_hover, params);
accel    = xdot(4:6);
omegadot = xdot(11:13);

tol = 1e-9;
if all(abs(accel) < tol) && all(abs(omegadot) < tol)
    fprintf('PASS: hover produces zero linear/angular acceleration\n');
else
    fprintf('FAIL: accel = [%.4g %.4g %.4g], omegadot = [%.4g %.4g %.4g]\n', ...
        accel(1),accel(2),accel(3), omegadot(1),omegadot(2),omegadot(3));
end

%% TEST 2 -- Free fall (zero thrust) vs. closed-form
fprintf('\n--- Test 2: Free fall vs analytic solution ---\n');
x0 = x_hover;
u0 = [0;0;0;0];
tspan = [0 2];
[t, X] = ode45(@(t,x) quadrotorDynamics(x, u0, params), tspan, x0);

z_sim      = X(:,3);
z_analytic = 0.5 * params.g * t.^2;
err = max(abs(z_sim - z_analytic));

figure('Name','Test 2: Free Fall');
plot(t, z_sim, 'b-', 'LineWidth', 2); hold on;
plot(t, z_analytic, 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('z position (NED, down = +) [m]');
legend('Simulated (quadrotorDynamics)', 'Analytic 0.5*g*t^2', 'Location','northwest');
title('Test 2: Free Fall Validation'); grid on;
saveas(gcf, 'test2_freefall.png');

fprintf('Max deviation from analytic free fall: %.3e m\n', err);
if err < 1e-3
    fprintf('PASS: matches analytic free fall\n');
else
    fprintf('FAIL: deviates from analytic free fall\n');
end

%% TEST 3 -- Pure roll torque: attitude isolation + resulting position drift
fprintf('\n--- Test 3: Roll torque response (visual) ---\n');
% Bias pattern chosen so tau_x (roll) != 0 while tau_y (pitch) and
% tau_z (yaw) both come out to exactly zero -- see the torque equations
% in quadrotorDynamics.m. Getting this bias pattern wrong (e.g. biasing
% the wrong rotor pair) silently tests pitch while a plot labeled "roll"
% is on screen, which is exactly the kind of bug this visual check exists
% to catch -- it caught one during development of this very script.
dT = 0.15; % [N] differential bias, hover +/- dT per rotor
u_roll = T_hover + dT*[-1; -1; 1; 1];

xdot_check = quadrotorDynamics(x0, u_roll, params);
fprintf('Isolation check -- angular accel [p q r]dot = [%.4g %.4g %.4g] rad/s^2\n', ...
    xdot_check(11), xdot_check(12), xdot_check(13));
fprintf('(only the first entry, roll, should be clearly nonzero)\n');

% tspan kept short enough that roll stays under 180 deg -- past that the
% atan2-based angle extraction wraps to -180, which would look like a
% discontinuity here even though nothing physically strange is happening.
% (This wraparound is exactly the yaw-flip issue -- same atan2() root
% cause -- your professor flagged for trajectory.m; see quat2eulerZYX.m.)
tspan = [0 0.65];
[t, X] = ode45(@(t,x) quadrotorDynamics(x, u_roll, params), tspan, x0);

roll_deg  = zeros(size(t));
pitch_deg = zeros(size(t));
yaw_deg   = zeros(size(t));
for i = 1:length(t)
    eul = quat2eulerZYX(X(i,7:10)');
    roll_deg(i)  = rad2deg(eul(1));
    pitch_deg(i) = rad2deg(eul(2));
    yaw_deg(i)   = rad2deg(eul(3));
end

figure('Name','Test 3: Roll Isolation', 'Position', [100 100 800 700]);

subplot(2,1,1);
plot(t, roll_deg,  'b-',  'LineWidth', 2.5); hold on;
plot(t, pitch_deg, 'r--', 'LineWidth', 1.8);
plot(t, yaw_deg,   'g-.', 'LineWidth', 1.8);
plot(t, zeros(size(t)), 'k:'); grid on;
xlabel('Time [s]'); ylabel('Angle [deg]');
legend('Roll (should grow)', 'Pitch (should stay 0)', 'Yaw (should stay 0)', ...
    'Location', 'northwest');
title('Test 3a: Attitude -- Only Roll Should Move');

subplot(2,1,2);
plot(t, X(:,1), 'b-', 'LineWidth', 2); hold on;   % x position, continuous line
plot(t, X(:,2), 'r-', 'LineWidth', 2);            % y position
plot(t, X(:,3), 'g-', 'LineWidth', 2);            % z position
grid on;
xlabel('Time [s]'); ylabel('Position (NED) [m]');
legend('x (should stay ~0)', 'y (drifts once tilted)', 'z (falls once tilted)', ...
    'Location', 'northwest');
title('Test 3b: Position Drift Caused By The Roll');

saveas(gcf, 'test3_roll_response.png');

fprintf('Inspect test3_roll_response.png:\n');
fprintf(' - Top: roll should rise smoothly away from 0; pitch and yaw\n');
fprintf('   should stay flat at 0 the whole time. If either of those\n');
fprintf('   moves, torque is leaking into the wrong axis -- go re-check\n');
fprintf('   the bias pattern against the torque equations by hand.\n');
fprintf(' - Bottom: x should stay ~0 (roll only rotates in the y-z plane);\n');
fprintf('   y and z should start drifting once the tilt is large enough\n');
fprintf('   that thrust noticeably points sideways instead of straight up.\n');
fprintf('   This is expected physical coupling, not a bug -- the total\n');
fprintf('   thrust magnitude never changed, only its direction did.\n');