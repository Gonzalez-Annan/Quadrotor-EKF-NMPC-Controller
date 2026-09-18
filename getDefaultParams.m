function params = getDefaultParams()
%GETDEFAULTPARAMS Single source of truth for physical/sensor parameters.
%   All of dynamics, EKF, and NMPC should pull constants from here so a
%   tuning change never has to be hunted down and repeated in three files.
%   Values are the nominal parameters from MA6224 Table 1 / Section 4.

    % --- Rigid body parameters (Table 1) ---
    params.m   = 1.20;              % total mass [kg]
    params.dx  = 0.225;             % rotor arm length, x [m]
    params.dy  = 0.225;             % rotor arm length, y [m]
    params.Jxx = 1.25e-2;           % roll inertia [kg m^2]
    params.Jyy = 1.25e-2;           % pitch inertia [kg m^2]
    params.Jzz = 2.20e-2;           % yaw inertia [kg m^2]
    params.J    = diag([params.Jxx, params.Jyy, params.Jzz]);
    params.Jinv = inv(params.J);
    params.ctau = 0.015;            % torque-to-thrust ratio [m]
    params.g    = 9.81;             % gravity [m/s^2], NED: acts in +z

    % --- Actuator / state constraints (Table 1) ---
    params.thrust_min_per_rotor = 0.2;   % [N]
    params.thrust_max_per_rotor = 5.5;   % [N]
    params.thrust_total_min     = 0.8;   % [N]
    params.thrust_total_max     = 22.0;  % [N]
    params.tilt_max_deg         = 35;    % max |roll|,|pitch| [deg]
    params.rate_max_deg         = [180, 180, 90]; % max |p|,|q|,|r| [deg/s]

    % --- Sensor characteristics (Section 4) ---
    params.imu_rate_hz = 100;
    params.gnss_rate_hz = 20;
    params.sigma_accel  = 0.08;   % [m/s^2]
    params.sigma_gyro   = 0.015;  % [rad/s]
    params.sigma_pos    = 0.02;   % [m]

    % ASSUMPTION: Section 4 states accel/gyro bias follow a random walk
    % (bdot_a = w_ba, bdot_g = w_bg) but does not give a numeric rate for
    % w_ba / w_bg. These two values are our own choice (typical small
    % MEMS-grade numbers) -- state this explicitly in the report rather
    % than presenting them as given. Units: per-sqrt-second, since a
    % random walk's variance grows linearly with time -> std grows with
    % sqrt(time).
    params.sigma_ba_rw = 0.001;   % [ (m/s^2) / sqrt(s) ] accel bias random-walk rate
    params.sigma_bg_rw = 0.0002;  % [ (rad/s) / sqrt(s) ] gyro bias random-walk rate

    % --- Trajectory parameters (Section 3) ---
    params.traj.Ax  = 3.0;
    params.traj.Ay  = 2.0;
    params.traj.Az  = 0.5;
    params.traj.z0  = -2.5;
    params.traj.w0  = 0.25;    % rad/s
    params.traj.tf  = 60;      % s
end