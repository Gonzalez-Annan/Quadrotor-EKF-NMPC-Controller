# MA6224 Quadrotor EKF-NMPC

Nonlinear state estimation (EKF) and constrained trajectory tracking (NMPC)
for a 6-DOF quadrotor tracking a 3D lemniscate. MA 6224: Unmanned Vehicles
course project.

## Run
```matlab
main_simulation
```

## Files
- `getDefaultParams.m` — all physical/sensor/NMPC constants, single source of truth
- `quadrotorDynamics.m` — 13-state nonlinear equations of motion
- `quat2rotmCustom.m`, `quatMultiplyCustom.m` — shared quaternion helpers
- `quat2eulerZYX.m` — quaternion to Euler angle conversion, for human-readable plots only
- `test_quadrotorDynamics.m` — validates dynamics before EKF/NMPC depend on it
- `generateTrajectory.m` — 3D lemniscate reference trajectory + analytic derivatives
- `test_generate_trajectory.m` — validates trajectory shape, period, derivatives, and yaw continuity
- `sensors.m`, `ekf.m`, `nmpc.m`, `plotting.m` — in progress
