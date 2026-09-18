function xdot = quadrotorDynamics(x, u, params)
%QUADROTORDYNAMICS Nonlinear 13-state quadrotor equations of motion.
%   Implements MA6224 Section 2.2. This is the single shared model used
%   by the true simulated dynamics, the EKF's prediction step, and the
%   NMPC's internal prediction model -- fix bugs here once, not three times.
%
%   xdot = quadrotorDynamics(x, u, params)
%
%   x (13x1) state, world/NED frame unless noted:
%       x(1:3)   p     - position [m]
%       x(4:6)   v     - velocity [m/s]
%       x(7:10)  q     - unit quaternion [qw qx qy qz], body->world (Hamilton)
%       x(11:13) omega - body angular rate [rad/s]
%
%   u (4x1): individual rotor thrusts [T0 T1 T2 T3] [N], rotor order as
%       defined in Section 2.1 (feeds directly into the torque equations
%       below -- do not silently reorder these).
%
%   params: struct from getDefaultParams() (needs m, J, Jinv, dx, dy,
%       ctau, g)

    v     = x(4:6);
    q     = x(7:10);
    omega = x(11:13);

    q = q / norm(q); % renormalize -- RK/ode integrators drift off the unit sphere

    T = u(:);
    Ttotal = sum(T);

    % Collective thrust acts along -z_body (NED: z is down, thrust pushes up)
    TB = [0; 0; -Ttotal];

    % Body torque from differential rotor thrust (Section 2.1)
    tau = [ params.dy * (-T(1) - T(2) + T(3) + T(4));
            params.dx * (-T(1) + T(2) + T(3) - T(4));
            params.ctau * (-T(1) + T(2) - T(3) + T(4)) ];

    R = quat2rotmCustom(q);     % body -> world rotation matrix
    g_world = [0; 0; params.g]; % NED: gravity acts in +z

    pdot     = v;
    vdot     = (1/params.m) * (R * TB) + g_world;
    qdot     = 0.5 * quatMultiplyCustom(q, [0; omega]);
    omegadot = params.Jinv * (tau - cross(omega, params.J * omega));

    xdot = [pdot; vdot; qdot; omegadot];
end
