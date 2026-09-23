function p=to_params(c)
%TO_PARAMS Translate the run configuration into the unchanged teammate API.
p.m=c.mass; p.g=c.g; p.dx=c.dx; p.dy=c.dy; p.ctau=c.ctau;
p.Jxx=c.J(1); p.Jyy=c.J(2); p.Jzz=c.J(3);
p.J=diag(c.J); p.Jinv=diag(1./c.J);
p.thrust_min_per_rotor=c.thrustMin; p.thrust_max_per_rotor=c.thrustMax;
p.thrust_total_min=4*c.thrustMin; p.thrust_total_max=4*c.thrustMax;
p.tilt_max_deg=c.tiltMax*180/pi; p.rate_max_deg=c.rateMax(:)'*180/pi;
p.imu_rate_hz=1/c.dt; p.gnss_rate_hz=1/c.gnssDt;
p.sigma_accel=c.sensor.accelSigma; p.sigma_gyro=c.sensor.gyroSigma;
p.sigma_pos=c.sensor.positionSigma;
p.sigma_ba_rw=c.sensor.accelBiasRW; p.sigma_bg_rw=c.sensor.gyroBiasRW;
p.traj=c.traj; p.traj.tf=c.duration; p.ekf=c.ekf;
end
