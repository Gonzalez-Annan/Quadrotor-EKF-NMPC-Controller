function eul = quat2eulerZYX(q)
%QUAT2EULERZYX Convert a unit quaternion [qw qx qy qz] to ZYX Euler
%   angles [roll; pitch; yaw] in radians (roll = rotation about x,
%   pitch = about y, yaw = about z). Standard aerospace convention.
%
%   This is for HUMAN-READABLE PLOTS ONLY. The actual dynamics, EKF, and
%   NMPC should keep working in quaternions internally -- Euler angles
%   have the well-known +/-180 deg wraparound (atan2) and a singularity
%   at +/-90 deg pitch (gimbal lock), neither of which the quaternion
%   representation suffers from.
%
%   eul = quat2eulerZYX(q)

    qw = q(1); qx = q(2); qy = q(3); qz = q(4);

    roll  = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx^2 + qy^2));

    sinp = 2*(qw*qy - qz*qx);
    sinp = max(min(sinp, 1), -1); % clamp: guards asin() near +/-90 deg pitch
    pitch = asin(sinp);

    yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));

    eul = [roll; pitch; yaw];
end
