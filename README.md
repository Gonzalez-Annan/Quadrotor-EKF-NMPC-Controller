# MA6224 Quadrotor EKF-NMPC

Nonlinear state estimation (EKF) and constrained trajectory tracking (NMPC)
for a 6-DOF quadrotor tracking a 3D lemniscate. MA 6224: Unmanned Vehicles
course project.

## Run
```matlab
main_simulation
```

## Files

### Shared / dynamics
- `getDefaultParams.m` — all physical/sensor/NMPC constants, single source of truth
- `quadrotorDynamics.m` — 13-state nonlinear equations of motion
- `quat2rotmCustom.m`, `quatMultiplyCustom.m` — shared quaternion helpers
- `quat2eulerZYX.m` — quaternion to Euler angle conversion, for human-readable plots only
- `test_quadrotorDynamics.m` — validates dynamics before EKF/NMPC depend on it

### Trajectory
- `generateTrajectory.m` — 3D lemniscate reference trajectory + analytic derivatives
- `test_generate_trajectory.m` — validates trajectory shape, period, derivatives, and yaw continuity

### Sensors
- `simulateIMU.m` — noisy accel/gyro measurements with bias random walk (Section 4)
- `simulateGNSS.m` — noisy GNSS position measurement (Section 4)
- `normpdfCustom.m` — standalone Gaussian PDF, used only by test_sensors.m plots
- `test_sensors.m` — validates IMU/GNSS noise statistics, bias drift, and sampling cadence

### EKF
- `ekf_init.m` — initializes the filter state and error covariance
- `ekf_predict.m` — IMU-driven propagation (state + error covariance), run at 100 Hz
- `ekf_update.m` — GNSS measurement correction with innovation gating, run at 20 Hz
- `ekf_state.m` — assembles the filter's estimate into the standard 13-state vector
- `skew3.m`, `quatLog.m`, `quatExp.m` — shared rotation-vector / quaternion helpers used by the EKF
- `estimation_error.m` — truth-minus-estimate error in the 15-dim tangent frame, used only for testing
- `test_ekf.m` — single-run ±2σ consistency check at hover
- `test_ekf_montecarlo.m` — same check averaged over many independent trials, since a single run's
  coverage statistic is noisy for weakly-observable states (gyro bias, z-axis accel bias at pure hover)

### In progress
- `nmpc.m`, `plotting.m`

## Assumptions worth noting in the report
- Section 4 specifies IMU bias follows a random walk but gives no numeric rate;
  `getDefaultParams.m` documents the assumed values (`sigma_ba_rw`, `sigma_bg_rw`)
  explicitly in its comments.
- The EKF's gyro bias and z-axis accelerometer bias states are weakly observable
  under a pure, unexcited hover — `test_ekf_montecarlo.m` shows their *mean*
  coverage is good (94%+) but with high trial-to-trial variance, consistent
  with known INS/GNSS behavior rather than an implementation defect. This is
  worth a sentence in the report's discussion of EKF tuning/limitations.
