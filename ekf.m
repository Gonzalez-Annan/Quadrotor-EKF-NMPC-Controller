function varargout=ekf(action,varargin)
%EKF Dispatcher for the original empty placeholder; params use teammate API.
% e=ekf('init',x,getDefaultParams(),true);
% e=ekf('predict',e,imu.accel,imu.gyro,dt,params);
% e=ekf('update',e,position,params); x=ekf('state',e);
setup_project();
switch lower(action)
    case 'init', f=str2func('ekf_init');
    case 'predict', f=str2func('ekf_predict');
    case 'update', f=str2func('ekf_update');
    case 'state', f=str2func('ekf_state');
    otherwise, error('merged:EKFAction','Unknown EKF action: %s',action);
end
[varargout{1:nargout}]=f(varargin{:});
end
