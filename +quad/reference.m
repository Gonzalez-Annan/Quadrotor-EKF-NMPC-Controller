function r=reference(t,c)
% Teammate trajectory + acceleration-based attitude/rate feedforward.
t=t(:)'; n=numel(t); p=integration.to_params(c);
traj=generateTrajectory(t,p);
r.p=traj.pos; r.v=traj.vel; r.a=traj.acc; r.yaw=traj.psi;
r.q=zeros(4,n); r.w=zeros(3,n);
for k=1:n
    R=attitude(t(k),p); r.q(:,k)=quad.Math.fromRot(R);
    h=1e-4; Rdot=(attitude(t(k)+h,p)-attitude(t(k)-h,p))/(2*h);
    W=R'*Rdot; r.w(:,k)=[W(3,2)-W(2,3);W(1,3)-W(3,1);W(2,1)-W(1,2)]/2;
end
r.x=[r.p;r.v;r.q;r.w];
end
function R=attitude(t,p)
a=generateTrajectory(t,p); psi=a.psi;
b3=[0;0;p.g]-a.acc; b3=b3/norm(b3);
b2=cross(b3,[cos(psi);sin(psi);0]); b2=b2/norm(b2);
R=[cross(b2,b3),b2,b3];
end
