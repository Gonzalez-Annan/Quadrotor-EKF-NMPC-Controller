function e=ekf_predict(e,accel,gyro,dt,c)
% IMU sample-average acceleration/gyro over this propagation interval.
a=accel-e.ba; w=gyro-e.bg;
qmid=quad.Math.mul(e.q,quad.Math.exp(w*dt/2)); R=quad.Math.rot(qmid);
aw=R*a+[0;0;c.g];
e.p=e.p+e.v*dt+0.5*aw*dt^2; e.v=e.v+aw*dt;
e.q=quad.Math.normalize(quad.Math.mul(e.q,quad.Math.exp(w*dt)));
e.omega=w;
F=zeros(15); F(1:3,4:6)=eye(3);
F(4:6,7:9)=-R*quad.Math.skew(a); F(4:6,10:12)=-R;
F(7:9,7:9)=-quad.Math.skew(w); F(7:9,13:15)=-eye(3);
Phi=eye(15)+F*dt+0.5*(F*dt)^2;
% Brief gives sample standard deviations, not continuous noise densities.
L=zeros(15,12); L(1:3,1:3)=0.5*R*dt^2; L(4:6,1:3)=R*dt;
L(7:9,4:6)=-eye(3)*dt;
L(10:12,7:9)=eye(3)*sqrt(dt); L(13:15,10:12)=eye(3)*sqrt(dt);
noise=[c.sensor.accelSigma^2*ones(3,1);c.sensor.gyroSigma^2*ones(3,1); ...
    c.sensor.accelBiasRW^2*ones(3,1);c.sensor.gyroBiasRW^2*ones(3,1)];
e.P=Phi*e.P*Phi'+L*diag(noise)*L'; e.P=(e.P+e.P')/2;
end
