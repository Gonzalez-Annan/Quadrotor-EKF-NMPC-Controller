%% test_quadrotorDynamics.m
% Visual + numeric validation of quadrotorDynamics.m, run BEFORE this
% model is trusted inside the EKF or NMPC. Three checks, each with an
% obvious pass/fail signal:
%   1. Hover equilibrium      -- numeric assert
%   2. Free fall              -- simulated vs. closed-form, plotted
%   3. Pure roll torque       -- angle history + 3D body-frame snapshots
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

%% TEST 3 -- Pure roll torque, visualized as a tipping body frame
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

tspan = [0 1.0];
[t, X] = ode45(@(t,x) quadrotorDynamics(x, u_roll, params), tspan, x0);

roll_deg = zeros(size(t));
for i = 1:length(t)
    q = X(i,7:10)';
    roll_deg(i) = rad2deg(atan2(2*(q(1)*q(2)+q(3)*q(4)), 1-2*(q(2)^2+q(3)^2)));
end

figure('Name','Test 3: Roll Response');
subplot(2,1,1);
plot(t, roll_deg, 'LineWidth', 2); grid on;
xlabel('Time [s]'); ylabel('Roll angle [deg]');
title('Test 3a: Roll Angle Under Differential Thrust');

subplot(2,1,2);
hold on; grid on; axis equal; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z (down)');
title('Test 3b: Body Frame Orientation Over Time (snapshots)');
sample_idx = round(linspace(1, length(t), 6));
colors = lines(numel(sample_idx));
for k = 1:numel(sample_idx)
    idx = sample_idx(k);
    q = X(idx,7:10)';
    R = quat2rotmCustom(q);
    origin = [0.3*(k-1); 0; 0]; % offset snapshots sideways so they don't overlap
    quiver3(origin(1),origin(2),origin(3), R(1,1),R(2,1),R(3,1), 0.2, ...
        'Color', colors(k,:), 'LineWidth', 2);       % body x-axis
    quiver3(origin(1),origin(2),origin(3), R(1,3),R(2,3),R(3,3), 0.2, ...
        'Color', colors(k,:)*0.5, 'LineWidth', 1);   % body z-axis
    text(origin(1),origin(2),origin(3)-0.05, sprintf('t=%.2fs', t(idx)), 'FontSize', 8);
end
saveas(gcf, 'test3_roll_response.png');

fprintf('Inspect test3_roll_response.png: the roll angle should move\n');
fprintf('monotonically away from zero and the body frames should visibly\n');
fprintf('tip over. If roll stays near 0 or tips the wrong way relative to\n');
fprintf('which rotors were biased up, the rotor sign convention needs a\n');
fprintf('second look before anything else is built on top of it.\n');
