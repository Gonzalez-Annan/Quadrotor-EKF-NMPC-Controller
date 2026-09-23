function e = ekf_update(e, position, params)
%EKF_UPDATE GNSS position fix. Call once per GNSS sample (20 Hz).
%
%   e = ekf_update(e, z, params)
%
%   Innovation gating: fixes whose NIS exceeds the chi-square gate are
%   rejected and COUNTED (report the count; do not silently drop).

gate = 16.27;  % chi-square 3-DOF ~99.9%
if isfield(params,'ekf') && isfield(params.ekf,'gnssGate')
    gate = params.ekf.gnssGate;
end

H = [eye(3), zeros(3,12)];
R = params.sigma_pos^2 * eye(3);

innovation = position(:) - e.p;
S = H*e.P*H' + R;
e.lastNIS = innovation'*(S\innovation);

if e.lastNIS > gate
    e.gnssRejected = e.gnssRejected + 1;
    return;
end

K = (e.P*H')/S;
delta = K*innovation;              % 15x1 correction in tangent coordinates

e.p  = e.p  + delta(1:3);
e.v  = e.v  + delta(4:6);
e.q  = quatMultiplyCustom(e.q, quatExp(delta(7:9)));   % inject, don't add
e.q  = e.q/norm(e.q);
e.ba = e.ba + delta(10:12);
e.bg = e.bg + delta(13:15);
e.omega = e.omega - delta(13:15);  % omega estimate = gyro - bg must follow

% Joseph-form covariance update (numerically stable)...
IKH = eye(15) - K*H;
P = IKH*e.P*IKH' + K*R*K';
% ...then covariance RESET for the injected attitude error: the error
% state is re-centered at the corrected quaternion, so its 3x3 attitude
% block transforms by G = I - skew(dtheta)/2 (standard error-state EKF).
G = eye(15);
G(7:9,7:9) = eye(3) - skew3(delta(7:9))/2;
e.P = G*P*G';
e.P = (e.P + e.P')/2;

e.gnssAccepted = e.gnssAccepted + 1;
end
