function report=run_merge_tests()
%RUN_MERGE_TESTS Verify both inherited numerics and the new integration seams.
setup_project(); c=project_config(); p=integration.to_params(c); rng(6224);
report.inherited=run_tests();
report.passed={};
for k=1:80
    x=[randn(6,1);quad.Math.exp(randn(3,1));randn(3,1)];
    u=c.thrustMin+(c.thrustMax-c.thrustMin)*rand(4,1);
    assert(norm(quadrotorDynamics(x,u,p)-quad.dynamics(x,u,c),inf)<1e-10);
    a=integration.plant_step(x,u,.01,p,5); b=quad.step(x,u,.01,c,5);
    assert(norm(a-b,inf)<1e-10,'Plant and prediction dynamics disagree.');
end
report.passed{end+1}='80 randomized plant/predictor derivative and RK4 comparisons';
t=0:.1:60; a=generateTrajectory(t,p); b=quad.reference(t,c);
assert(norm(a.pos-b.p,'fro')<1e-12 && norm(a.vel-b.v,'fro')<1e-12);
assert(norm(a.acc-b.a,'fro')<1e-12 && norm(a.psi-b.yaw)<1e-12);
% Changing the teammate trajectory configuration must reach the NMPC reference.
d=c; d.traj.Ax=1.7; d.traj.w0=.31;
pd=integration.to_params(d); a=generateTrajectory(t,pd); b=quad.reference(t,d);
assert(norm(a.pos-b.p,'fro')<1e-12);
report.passed{end+1}='Trajectory passthrough including non-default parameters';
assert(p.sigma_ba_rw==getDefaultParams().sigma_ba_rw);
r=quad.reference(0,c); x=r.x; direct=ekf_init(x,p,true); adapted=quad.ekf_init(x,c,true);
for k=1:100
    acc=[.01;-.02;-c.g]+.01*randn(3,1); gyro=.01*randn(3,1);
    direct=ekf_predict(direct,acc,gyro,c.dt,p);
    adapted=quad.ekf_predict(adapted,acc,gyro,c.dt,c);
    if mod(k,5)==0
        pos=x(1:3)+.01*randn(3,1);
        direct=ekf_update(direct,pos,p); adapted=quad.ekf_update(adapted,pos,c);
    end
    assert(norm(ekf_state(direct)-quad.ekf_state(adapted))<1e-12);
    assert(norm(direct.P-adapted.P,'fro')<1e-12);
    assert(min(eig(adapted.P))>-1e-12);
end
report.passed{end+1}='100-step direct/adapter EKF equality and positive covariance';
% The original sensor returns the advanced bias in the measurement itself.
p.sigma_accel=0; p.sigma_gyro=0;
u=c.hover; dx=quadrotorDynamics(x,u,p);
[imu,ba,bg]=simulateIMU(x,dx,zeros(3,1),zeros(3,1),c.dt,p);
assert(norm(imu.accel-[0;0;-sum(u)/p.m]-ba)<1e-10);
assert(norm(imu.gyro-x(11:13)-bg)<1e-12);
assert(norm(simulateGNSS(x,setfield(p,'sigma_pos',0))-x(1:3))<1e-12); %#ok<SFLD>
report.passed{end+1}='Sensor specific force and returned-bias timing';
report.matlab=version; report.time=char(datetime('now'));
disp(report.passed');
end
