function e=ekf_update(e,position,c)
f=str2func('ekf_update'); e=f(e,position,integration.to_params(c));
end
