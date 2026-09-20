function e=ekf_init(x,c,addError)
% Multiplicative error-state EKF: nominal [p v q ba bg], 15x15 covariance.
% Error ordering [dp dv dtheta dba dbg]; right/body attitude perturbation.
if nargin<3, addError=false; end
e.p=x(1:3); e.v=x(4:6); e.q=x(7:10); e.ba=zeros(3,1); e.bg=zeros(3,1);
if addError
    e.p=e.p+c.ekf.initialPositionError; e.v=e.v+c.ekf.initialVelocityError;
    e.q=quad.Math.mul(e.q,quad.Math.exp(c.ekf.initialAttitudeError));
end
e.P=diag(c.ekf.initialStd.^2); e.omega=x(11:13);
e.gnssRejected=0; e.gnssAccepted=0; e.lastNIS=NaN;
end
