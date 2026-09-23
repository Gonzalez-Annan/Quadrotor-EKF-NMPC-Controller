function g=state_constraints(x,c)
% g<=0. Separate roll/pitch bounds, not a single total-tilt constraint.
q=x(7:10,:); w=x(11:13,:);
R31=2*(q(2,:).*q(4,:)-q(1,:).*q(3,:));
R32=2*(q(3,:).*q(4,:)+q(1,:).*q(2,:));
R33=1-2*(q(2,:).^2+q(3,:).^2);
g=[R32-tan(c.tiltMax)*R33;-R32-tan(c.tiltMax)*R33; ...
    R31-sin(c.tiltMax);-R31-sin(c.tiltMax); ...
    w./c.rateMax-1;-w./c.rateMax-1];
end
