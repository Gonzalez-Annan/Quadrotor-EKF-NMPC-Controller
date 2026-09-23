function r=ros_reference(t,c)
% Initial hover -> C2 quintic transition -> exact required 60-s mission.
t=t(:)'; n=numel(t); r.p=zeros(3,n); r.v=r.p; r.a=r.p;
r.q=zeros(4,n); r.w=zeros(3,n); r.yaw=zeros(1,n);
for k=1:n
    [r.p(:,k),r.v(:,k),r.a(:,k),r.yaw(k),R]=sample(t(k),c);
    r.q(:,k)=quad.Math.fromRot(R); h=1e-4;
    [~,~,~,~,Ra]=sample(t(k)-h,c); [~,~,~,~,Rb]=sample(t(k)+h,c);
    D=R'*(Rb-Ra)/(2*h); r.w(:,k)=[D(3,2)-D(2,3);D(1,3)-D(3,1);D(2,1)-D(1,2)]/2;
end
r.x=[r.p;r.v;r.q;r.w];
end
function [p,v,a,yaw,R]=sample(t,c)
start=c.ros.warmupSeconds; T=c.ros.transitionSeconds;
r0=quad.reference(0,c); yaw=r0.yaw;
if t<start
    p=c.ros.initialPosition; v=zeros(3,1); a=v;
elseif t<start+T
    s=t-start; p0=c.ros.initialPosition;
    M=[T^3 T^4 T^5;3*T^2 4*T^3 5*T^4;6*T 12*T^2 20*T^3];
    coeff=M\[(r0.p-p0)';r0.v';r0.a'];
    p=p0+coeff'*[s^3;s^4;s^5]; v=coeff'*[3*s^2;4*s^3;5*s^4];
    a=coeff'*[6*s;12*s^2;20*s^3];
else
    rr=quad.reference(t-start-T,c); p=rr.p; v=rr.v; a=rr.a; yaw=rr.yaw;
end
b3=[0;0;c.g]-a; b3=b3/norm(b3); b2=cross(b3,[cos(yaw);sin(yaw);0]);
b2=b2/norm(b2); R=[cross(b2,b3),b2,b3];
end
