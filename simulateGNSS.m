function pos_meas = simulateGNSS(x_true, params)
%SIMULATEGNSS Simulated GNSS position fix (Section 4).
%   Plays the role of GNSS HARDWARE. Unlike the IMU, Section 4 gives no
%   bias/random-walk term for GNSS -- just white measurement noise -- so
%   this function is stateless and needs no bias carried between calls.
%
%   pos_meas = simulateGNSS(x_true, params)
%
%   x_true : 13x1 true state
%   params : struct from getDefaultParams() (needs params.sigma_pos)
%
%   Returns pos_meas, 3x1, the noisy position fix in the world frame.

    p_true = x_true(1:3);
    pos_meas = p_true + params.sigma_pos * randn(3,1);
end