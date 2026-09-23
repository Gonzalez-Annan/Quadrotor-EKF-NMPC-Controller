function v = quatLog(q)
%QUATLOG Quaternion logarithm: unit quaternion -> rotation vector [rad].
%   Returns the rotation on the branch with qw >= 0 (q and -q are the
%   same rotation; this keeps the result continuous for plots/errors).
    q = q/norm(q);
    if q(1) < 0, q = -q; end
    s = norm(q(2:4));
    if s < 1e-10
        v = 2*q(2:4);
    else
        v = 2*atan2(s, q(1)) * q(2:4)/s;
    end
end
