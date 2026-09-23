function S = skew3(v)
%SKEW3 Skew-symmetric cross-product matrix: S*w == cross(v,w).
    S = [   0  -v(3)  v(2);
         v(3)     0  -v(1);
        -v(2)  v(1)    0 ];
end
