function [W,B]=frames(c)
% p_NED=W*p_external; v_FRD=B*v_external_body.
switch upper(c.ros.worldFrame)
    case 'ENU', W=[0 1 0;1 0 0;0 0 -1];
    case 'NED', W=eye(3);
    otherwise, error('worldFrame must be ENU or NED.');
end
switch upper(c.ros.bodyFrame)
    case 'FLU', B=diag([1 -1 -1]);
    case 'FRD', B=eye(3);
    otherwise, error('bodyFrame must be FLU or FRD.');
end
end
