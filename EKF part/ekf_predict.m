function e = ekf_predict(e, accel, gyro, dt, params)
%EKF_PREDICT IMU-driven propagation. Call once per IMU sample (100 Hz).
%
%   e = ekf_predict(e, imu.accel, imu.gyro, dt, params)
%
%   The accelerometer measures body-frame SPECIFIC FORCE (see simulateIMU.m):
%   after bias correction it is rotated into the world frame with the
%   HALF-STEP attitude and gravity is added back (NED: +z).

a = accel(:) - e.ba;   % bias-corrected specific force, body frame
w = gyro(:)  - e.bg;   % bias-corrected body rate

% half-step attitude rotates the specific force over the interval
qmid = quatMultiplyCustom(e.q, quatExp(w*dt/2));
qmid = qmid/norm(qmid);
R = quat2rotmCustom(qmid);              % body -> world
aw = R*a + [0; 0; params.g];            % world-frame acceleration, NED

% second-order position integration; exact exponential attitude update
e.p = e.p + e.v*dt + 0.5*aw*dt^2;
e.v = e.v + aw*dt;
e.q = quatMultiplyCustom(e.q, quatExp(w*dt));
e.q = e.q/norm(e.q);
e.omega = w;   % gyro directly measures omega; bias handled via bg

% --- error-state covariance propagation ---
% Nonzero blocks of the continuous-time error dynamics F (15x15):
%   dp_dot     = dv
%   dv_dot     = -R*skew(a)*dtheta - R*dba + n_a
%   dtheta_dot = -skew(w)*dtheta - dbg + n_w
%   dba_dot    = n_ba        dbg_dot = n_bg
F = zeros(15);
F(1:3,4:6)   = eye(3);
F(4:6,7:9)   = -R*skew3(a);
F(4:6,10:12) = -R;
F(7:9,7:9)   = -skew3(w);
F(7:9,13:15) = -eye(3);
Fdt = F*dt;
Phi = eye(15) + Fdt + 0.5*Fdt^2;   % include 2nd-order term: w*dt up to ~0.9 deg/s*0.01s is small but cheap to keep

% Noise injection matrix L (15x12) + per-sample variances from params.
% Section 4 gives per-SAMPLE stds for accel/gyro and per-sqrt-second bias
% random-walk rates (see getDefaultParams comments), hence the dt scalings:
% accel white noise -> velocity ~ dt, position ~ dt^2/2;
% bias random walk -> variance grows linearly with dt (std ~ sqrt(dt)).
L = zeros(15,12);
L(1:3,1:3)     = 0.5*R*dt^2;
L(4:6,1:3)     = R*dt;
L(7:9,4:6)     = eye(3)*dt;
L(10:12,7:9)   = eye(3)*sqrt(dt);
L(13:15,10:12) = eye(3)*sqrt(dt);
noise = [params.sigma_accel^2*ones(3,1);   params.sigma_gyro^2*ones(3,1); ...
         params.sigma_ba_rw^2*ones(3,1);    params.sigma_bg_rw^2*ones(3,1)];

e.P = Phi*e.P*Phi' + L*diag(noise)*L';
e.P = (e.P + e.P')/2;              % symmetrize (long-run drift guard)
end
