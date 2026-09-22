function e = ekf_init(x, params, addError)
%EKF_INIT Multiplicative error-state EKF, first-codebase interface.
%   Nominal state (stored in struct e): [p; v; q; ba; bg] (16 components).
%   Covariance e.P is 15x15, defined on the error state
%       [dp; dv; dtheta; dba; dbg]
%   with a RIGHT (body-frame) attitude perturbation:
%       q_true = q_hat (x) Exp(dtheta)
%   (Exp/Log here are quatExp/quatLog; the unit quaternion has only 3
%   independent error directions, so no 4th attitude covariance column.)
%
%   e = ekf_init(x, params)            % exact initialization from 13-state x
%   e = ekf_init(x, params, true)      % add small configured initial errors
%
%   Optional overrides: add a params.ekf struct in getDefaultParams with any
%   of the field names listed in ekfDefaults below.

if nargin < 3, addError = false; end
s = ekfDefaults(params);

e.p = x(1:3);
e.v = x(4:6);
e.q = x(7:10);  e.q = e.q/norm(e.q);
e.ba = zeros(3,1);
e.bg = zeros(3,1);

if addError
    e.p = e.p + s.initialPositionError;
    e.v = e.v + s.initialVelocityError;
    e.q = quatMultiplyCustom(e.q, quatExp(s.initialAttitudeError));
    e.q = e.q/norm(e.q);
end

e.P = diag(s.initialStd.^2);   % 15x15, error-state ordering above
e.omega = x(11:13);            % controller input uses gyro-minus-bias
e.gnssRejected = 0;
e.gnssAccepted = 0;
e.lastNIS = NaN;
end

function s = ekfDefaults(params)
%EKFDEFAULTS Initial-error sizes, initial std, and the GNSS innovation gate.
%   Values mirror the validated reference implementation. Add a params.ekf
%   struct in getDefaultParams to override any field (e.g. for a report
%   sensitivity study) without editing this file.
s.initialStd = [0.05*ones(3,1);      % dp   [m]
                0.1*ones(3,1);       % dv   [m/s]
                2*pi/180*ones(3,1);  % dtheta [rad]
                0.08*ones(3,1);      % dba  [m/s^2]
                0.008*ones(3,1)];    % dbg  [rad/s]
s.initialPositionError   = [0.015; -0.01; 0.01];
s.initialVelocityError   = [0.02; -0.01; 0.01];
s.initialAttitudeError   = [0.3; -0.3; 0.5]*pi/180;
s.gnssGate = 16.27;  % chi-square, 3 DOF, ~99.9%: reject wild position fixes
if isfield(params, 'ekf')
    f = fieldnames(params.ekf);
    for i = 1:numel(f)
        s.(f{i}) = params.ekf.(f{i});
    end
end
end
