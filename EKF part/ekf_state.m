function x = ekf_state(e)
%EKF_STATE Assemble the 13-state controller input [p; v; q; omega].
x = [e.p; e.v; e.q; e.omega];
end
