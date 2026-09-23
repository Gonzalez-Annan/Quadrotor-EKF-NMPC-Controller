function e=ekf_init(x,c,addError)
% Adapter to the unchanged teammate implementation, not a second EKF.
if nargin<3, addError=false; end
f=str2func('ekf_init'); e=f(x,integration.to_params(c),addError);
end
