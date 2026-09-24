function report=run_tests()
% Meaningful numerical checks of physics, geometry, derivatives, and EKF.
c=project_config(); tests={@physics,@rotations,@trajectory,@derivatives,@filter,@controller,@constraintBounds};
names={'physics','rotations','trajectory','MPC gradients','EKF covariance','NMPC hover','Constraint boundaries'};
for k=1:numel(tests)
    ticID=tic; tests{k}(c); fprintf('PASS %-20s %.2f s\n',names{k},toc(ticID));
end
report.passed=names; report.matlab=version; report.time=char(datetime('now'));
end
function constraintBounds(c)
x=[zeros(6,1);quad.Math.fromEuler(30*pi/180,30*pi/180,0);zeros(3,1)];
assert(max(quad.state_constraints(x,c))<0,'Separate roll/pitch limits incorrectly treated as total tilt.');
x(7:10)=quad.Math.fromEuler(c.tiltMax,0,0);
assert(abs(max(quad.state_constraints(x,c)))<1e-12);
x(7:10)=quad.Math.fromEuler(c.tiltMax+.001,0,0);
assert(max(quad.state_constraints(x,c))>0);
x(7:10)=[1;0;0;0]; x(11:13)=c.rateMax;
assert(abs(max(quad.state_constraints(x,c)))<1e-12);
x(13)=1.01*c.rateMax(3); assert(max(quad.state_constraints(x,c))>0);
end
function physics(c)
x=[zeros(6,1);1;zeros(6,1)];
assert(norm(quad.dynamics(x,c.hover,c))<1e-12,'Hover is not an equilibrium.');
d=quad.dynamics(x,zeros(4,1),c); assert(abs(d(6)-c.g)<1e-12);
u=c.hover+[.1;.1;-.1;-.1]; d=quad.dynamics(x,u,c);
assert(norm(c.mix*c.mpc.inputMap-diag([c.mass;c.J]),'fro')<1e-12);
assert(d(11)<0 && abs(d(12))<1e-10 && abs(d(13))<1e-10,'Mixer mismatch.');
x(11:13)=[.2;-.3;.1]; y=quad.step(x,c.hover,.5,c,100);
assert(abs(norm(y(7:10))-1)<1e-12);
end
function rotations(~)
for k=1:20
    q=quad.Math.exp([.04*k;-.02*k;.1*k]); R=quad.Math.rot(q); z=quad.Math.fromRot(R);
    assert(norm(R'*R-eye(3),'fro')<1e-12 && abs(det(R)-1)<1e-12);
    assert(abs(abs(q'*z)-1)<1e-12);
end
end
function trajectory(c)
t=7.3; h=1e-4; a=quad.reference(t-h,c); b=quad.reference(t+h,c); r=quad.reference(t,c);
assert(norm((b.p-a.p)/(2*h)-r.v)<1e-8);
assert(norm((b.v-a.v)/(2*h)-r.a)<1e-8);
end
function derivatives(c)
c.mpc.horizon=4; c.mpc.blocks=[1 1 2];
r0=quad.reference(.7,c); r=quad.reference(.7+(1:4)*c.controlDt,c);
z=.01*sin((1:12)');
[~,~,dj,dg]=quad.mpc_problem(z,r0.x,c.hover,r,c);
h=1e-5;
for k=[1 3 6 11]
    d=zeros(12,1); d(k)=h;
    [ja,ga]=quad.mpc_problem(z+d,r0.x,c.hover,r,c);
    [jb,gb]=quad.mpc_problem(z-d,r0.x,c.hover,r,c);
    assert(abs((ja-jb)/(2*h)-dj(k))<1e-4);
    assert(max(abs((ga-gb)/(2*h)-dg(k,:)'))<1e-4);
end
end
function filter(c)
x=[0;0;-2;zeros(3,1);1;zeros(6,1)]; e=quad.ekf_init(x,c,true);
initial=norm(e.p-x(1:3));
for k=1:100
    e=quad.ekf_predict(e,[0;0;-c.g],zeros(3,1),.01,c);
    if mod(k,5)==0, e=quad.ekf_update(e,x(1:3),c); end
end
assert(norm(e.p-x(1:3))<initial);
assert(min(eig(e.P))>-1e-12 && norm(e.P-e.P','fro')<1e-12);
assert(abs(norm(e.q)-1)<1e-12);
end
function controller(c)
x=[0;0;-2;zeros(3,1);1;zeros(6,1)];
r.p=repmat(x(1:3),1,c.mpc.horizon); r.v=zeros(3,c.mpc.horizon);
r.q=repmat(x(7:10),1,c.mpc.horizon); r.w=zeros(3,c.mpc.horizon);
ctrl=quad.mpc_init(c); [u,~,info]=quad.mpc_step(x,0,ctrl,c,r);
assert(~info.fallback && norm(u-c.hover)<1e-3);
end
