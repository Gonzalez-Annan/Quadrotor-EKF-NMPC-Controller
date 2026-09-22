function d = estimation_error(e, x_true, bias)
%ESTIMATION_ERROR Truth-minus-nominal error in the 15-dim tangent frame.
%   d = [dp; dv; dtheta; dba; dbg], where the attitude part is
%   Log(q_hat^-1 (x) q_true) -- the LOCAL rotation vector. Raw quaternion
%   subtraction is wrong here because q and -q are the same rotation.
    qc = [e.q(1); -e.q(2:4)];   % conjugate
    d = [x_true(1:3) - e.p;
         x_true(4:6) - e.v;
         quatLog(quatMultiplyCustom(qc, x_true(7:10)));
         bias(1:3) - e.ba;
         bias(4:6) - e.bg];
end
