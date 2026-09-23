function x=plant_step(x,u,dt,p,n)
%PLANT_STEP RK4 around the teammate's original scalar dynamics function.
h=dt/n;
for k=1:n
    a=quadrotorDynamics(x,u,p);
    b=quadrotorDynamics(x+h*a/2,u,p);
    d=quadrotorDynamics(x+h*b/2,u,p);
    e=quadrotorDynamics(x+h*d,u,p);
    x=x+h*(a+2*b+2*d+e)/6;
    x(7:10)=x(7:10)/norm(x(7:10));
end
end
