function R = quat2rotmCustom(q)
%QUAT2ROTMCUSTOM Body -> world rotation matrix from a unit quaternion.
%   q is [qw qx qy qz] (Hamilton convention). Written as a standalone
%   function (no toolbox dependency) so it works identically in plain
%   MATLAB, MATLAB with any toolbox, and Octave.
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    R = [1-2*(qy^2+qz^2),   2*(qx*qy-qw*qz),   2*(qx*qz+qw*qy);
         2*(qx*qy+qw*qz),   1-2*(qx^2+qz^2),   2*(qy*qz-qw*qx);
         2*(qx*qz-qw*qy),   2*(qy*qz+qw*qx),   1-2*(qx^2+qy^2)];
end
