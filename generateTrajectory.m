function traj = generateTrajectory(t, params)
%GENERATETRAJECTORY 3D lemniscate (figure-8) reference trajectory.
%   Implements MA6224 Section 3, Eqs. 1-4, plus their analytic
%   derivatives (needed by the NMPC's reference feedforward).
%
%   traj = generateTrajectory(t, params)
%
%   t: 1xN (or Nx1) vector of time samples [s]
%   params: struct from getDefaultParams() (needs params.traj.*)
%
%   Returns a struct traj with fields, each 3xN unless noted:
%       t          - time samples, 1xN (row, for consistent plotting)
%       pos        - [xr; yr; zr]        reference position [m]
%       vel        - [xdot; ydot; zdot]  reference velocity [m/s]
%       acc        - [xddot; yddot; zddot] reference acceleration [m/s^2]
%       psi        - reference yaw, UNWRAPPED (continuous), 1xN [rad]
%       psi_raw    - reference yaw, raw atan2 output (has +/-180 deg
%                    jumps), 1xN [rad] -- kept only so the discontinuity
%                    can be demonstrated/tested; nothing downstream
%                    should consume this field, use psi instead.

t = t(:)'; % force row vector, so every field below is 1xN consistently

Ax = params.traj.Ax;
Ay = params.traj.Ay;
Az = params.traj.Az;
z0 = params.traj.z0;
w0 = params.traj.w0;

% --- Position (Eqs. 1-3) ---
xr = Ax * sin(w0*t);
yr = Ay * sin(2*w0*t);
zr = z0 + Az * cos(w0*t);

% --- Velocity (analytic d/dt of the above) ---
xdot = Ax * w0 * cos(w0*t);
ydot = Ay * 2*w0 * cos(2*w0*t);
zdot = -Az * w0 * sin(w0*t);

% --- Acceleration (analytic d/dt of velocity) ---
xddot = -Ax * w0^2 * sin(w0*t);
yddot = -Ay * (2*w0)^2 * sin(2*w0*t);
zddot = -Az * w0^2 * cos(w0*t);

% --- Yaw (Eq. 4) ---
% atan2 alone is only defined on (-180, 180] deg, so as the heading
% sweeps all the way around (this path does ~2.4 full loops over the
% mission), the raw signal jumps by ~360 deg at the wrap boundary.
% unwrap() removes that artificial jump so downstream consumers
% (e.g. a yaw-rate feedforward, or any EKF/NMPC yaw error) see a
% continuous signal. See test_generateTrajectory.m Test 4 for a
% direct before/after demonstration.
psi_raw = atan2(ydot, xdot);
psi     = unwrap(psi_raw);

traj.t       = t;
traj.pos     = [xr; yr; zr];
traj.vel     = [xdot; ydot; zdot];
traj.acc     = [xddot; yddot; zddot];
traj.psi     = psi;
traj.psi_raw = psi_raw;
end