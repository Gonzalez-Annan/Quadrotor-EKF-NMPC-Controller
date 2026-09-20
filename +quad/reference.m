function r=reference(t,c)
% Required figure-eight, with acceleration-based attitude feedforward.
t=t(:)'; n=numel(t);
r.p=[3*sin(.25*t);2*sin(.5*t);-2.5+.5*cos(.25*t)];
r.v=[.75*cos(.25*t);cos(.5*t);-.125*sin(.25*t)];
r.a=[-.1875*sin(.25*t);-.5*sin(.5*t);-.03125*cos(.25*t)];
r.yaw=atan2(r.v(2,:),r.v(1,:)); r.q=zeros(4,n); r.w=zeros(3,n);
for k=1:n
    R=attitude(t(k),c); r.q(:,k)=quad.Math.fromRot(R);
    h=1e-4; Rdot=(attitude(t(k)+h,c)-attitude(t(k)-h,c))/(2*h);
    W=R'*Rdot; r.w(:,k)=[W(3,2)-W(2,3);W(1,3)-W(3,1);W(2,1)-W(1,2)]/2;
end
r.x=[r.p;r.v;r.q;r.w];
end
function R=attitude(t,c)
a=[-.1875*sin(.25*t);-.5*sin(.5*t);-.03125*cos(.25*t)];
psi=atan2(cos(.5*t),.75*cos(.25*t));
b3=[0;0;c.g]-a; b3=b3/norm(b3);
b2=cross(b3,[cos(psi);sin(psi);0]); b2=b2/norm(b2);
R=[cross(b2,b3),b2,b3];
end
