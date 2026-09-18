function qout = quatMultiplyCustom(q, r)
%QUATMULTIPLYCUSTOM Hamilton product q (x) r, both quaternions [w x y z].
%   Standalone (no toolbox dependency): works identically in plain
%   MATLAB and Octave.
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    rw = r(1); rx = r(2); ry = r(3); rz = r(4);
    qout = [ qw*rw - qx*rx - qy*ry - qz*rz;
             qw*rx + qx*rw + qy*rz - qz*ry;
             qw*ry - qx*rz + qy*rw + qz*rx;
             qw*rz + qx*ry - qy*rx + qz*rw ];
end
