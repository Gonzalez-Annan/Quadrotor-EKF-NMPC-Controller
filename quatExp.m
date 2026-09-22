function q = quatExp(v)
%QUATEXP Quaternion exponential: rotation vector v [rad] -> unit quaternion.
%   Inverse of quatLog. Small-angle safe. Convention matches
%   quatMultiplyCustom (Hamilton, scalar first).
    a = norm(v);
    if a < 1e-8
        q = [1; v/2];
        q = q/norm(q);
    else
        q = [cos(a/2); sin(a/2)*(v/a)];
    end
end
