function e=ekf_predict(e,accel,gyro,dt,c)
f=str2func('ekf_predict'); e=f(e,accel,gyro,dt,integration.to_params(c));
end
