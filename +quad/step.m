function x=step(x,u,dt,c,n)
% RK4 integration with quaternion normalization after every substep.
if nargin<5, n=1; end
h=dt/n;
for k=1:n
    a=quad.dynamics(x,u,c); b=quad.dynamics(x+h*a/2,u,c);
    d=quad.dynamics(x+h*b/2,u,c); e=quad.dynamics(x+h*d,u,c);
    x=x+h*(a+2*b+2*d+e)/6;
    x(7:10,:)=quad.Math.normalize(x(7:10,:));
end
end
