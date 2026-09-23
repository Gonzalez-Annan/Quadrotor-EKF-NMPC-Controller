function dx=dynamics(x,u,c)
% Vectorized 13-state rigid-body model; x and u have matching column counts.
q=quad.Math.normalize(x(7:10,:)); w=x(11:13,:);
thrust=sum(u,1); tau=c.mix(2:4,:)*u;
zBody=[2*(q(2,:).*q(4,:)+q(1,:).*q(3,:)); ...
       2*(q(3,:).*q(4,:)-q(1,:).*q(2,:)); ...
       1-2*(q(2,:).^2+q(3,:).^2)];
acc=-zBody.*(thrust/c.mass)+[0;0;c.g];
qdot=0.5*quad.Math.mul(q,[zeros(1,size(w,2));w]);
wdot=(tau-cross(w,c.J.*w,1))./c.J;
dx=[x(4:6,:);acc;qdot;wdot];
end
