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
- `simulateIMU.m` — noisy accel/gyro measurements with bias random walk (Section 4)
- `simulateGNSS.m` — noisy GNSS position measurement (Section 4)
- `normpdfCustom.m` — standalone Gaussian PDF, used only by test_sensors.m plots
- `test_sensors.m` — validates IMU/GNSS noise statistics, bias drift, and sampling cadence
- `ekf.m`, `nmpc.m`, `plotting.m` — in progress

## Assumptions worth noting in the report
- Section 4 specifies IMU bias follows a random walk but gives no numeric rate;
  `getDefaultParams.m` documents the assumed values (`sigma_ba_rw`, `sigma_bg_rw`)
  explicitly in its comments.
