# Validation Record

Validation was performed on the user's computer in MATLAB R2026a Update 4 (`26.1.0.3312084`), with Optimization Toolbox and ROS Toolbox available. Tests were executed through an existing MATLAB session. The reported results came from actual MATLAB execution, not a substitute simulation in Python.

## Completed 60-Second Experiments

All experiments used the prescribed mass, inertia, sensor frequencies, figure-eight trajectory, and actuator, attitude, and angular-rate limits. Each run contains 6,001 state samples and 1,200 NMPC updates.

| Experiment | Position RMSE (cm) | Maximum position error (cm) | Mean solve time (ms) | Maximum solve time (ms) | Fallback input count |
|---|---:|---:|---:|---:|---:|
| Ground-truth feedback baseline | 0.87 | 2.26 | 34.2 | 88.5 | 0 |
| EKF feedback, seed 6224 | 3.87 | 9.70 | 69.9 | 166.4 | 0 |
| EKF feedback, seed 6225 | 3.47 | 7.16 | 69.4 | 169.6 | 0 |

All three runs recorded zero rotor-thrust, roll/pitch, and body-rate constraint violation samples. All 1,200 solves in each run exited normally. These are checks at recorded sampling points, not a mathematical proof of continuous-time constraint satisfaction.

The primary example is in `results/validated_60s/`, the additional noise-seed experiment is in `results/ekf_seed_6225/`, and the baseline is in `results/baseline_truth/`. Exact values are available in each run's `metrics.json` and in `results/validation_comparison.csv`.

## Numerical and Interface Tests

The following checks were completed:

- Hover equilibrium, zero-thrust free fall, and differential-thrust directions.
- Rotation-matrix orthogonality, quaternion round-trip conversion, and unit norm.
- Agreement between analytical reference velocity/acceleration and independent central finite differences.
- Agreement between complex-step NMPC objective/constraint gradients and independent central finite differences.
- EKF position correction, covariance symmetry, and positive semidefiniteness.
- Recovery of the known optimal hover command.
- ENU/FLU ↔ NED/FRD coordinate transformations.
- Translational continuity of the Gazebo startup reference.
- Separate roll/pitch boundaries, angular-rate boundaries, and violation detection.
- A **local ROS 2 loopback test** using real publisher/subscriber nodes and synthetic IMU, position, ground-truth, and thrust messages. Data and coordinate conversions were checked using domain ID 198 and dedicated test topics, without an external Gazebo instance.

Users can rerun `run_tests` and `test_ros_interface`. Final numerical test records are stored in `results/test_report.mat` and `results/validation_checks.mat`; the ROS loopback record is stored in `results/ros_report.mat`.

All nine numerical self-tests passed. MATLAB Code Analyzer checked 29 `.m` files and reported no issues. A known synthetic trajectory was also used to validate Gazebo log timestamp alignment and postprocessing, producing the expected zero error. This was not an external Gazebo flight test.

The complete MP4 was exported and reopened with MATLAB `VideoReader`: duration 60.05 seconds, resolution 1334×750, and frame rate 20 fps. It satisfies the maximum duration of two minutes. Example plots and video frames were visually inspected.

## Interpretation of Results

For seed 6224, EKF position-estimation RMSE was approximately 1.39 cm. Empirical ±2σ coverage for the three position axes was approximately 95.2%, 95.5%, and 95.3%. Coverage for some bias components was below 95%, while other bias and heading components were more conservative. A single trajectory therefore does not establish rigorous uncertainty calibration for every state.

The GNSS innovation gate rejected four position updates for seed 6224 and one for seed 6225. These counts are retained in the logs. The baseline also simulates sensors and runs the filter, but its controller receives ground truth; baseline tracking performance must not be presented as estimated-state feedback performance.

## Known Limitations

1. **External Gazebo integration has not been completed.** The SDF model, sensors, ROS bridge, and rotor-thrust adapter belong to the external setup. Passing a local ROS loopback test does not establish successful flight of the external quadrotor model.
2. **Sustained real-time control at 20 Hz has not been achieved.** The default control period is 50 ms. For seed 6224, 961 of 1,200 solves exceeded that period, with an average solve time of approximately 70 ms. Gazebo must be slowed down with additional communication margin, or the implementation/solver must be optimized further.
3. The standalone simulation starts at the trajectory's initial state, including nonzero velocity. It is not a ground-takeoff experiment. The Gazebo entry point provides a separate startup transition, but that online transition has not yet been validated in external Gazebo.
4. Bias random walks, initial errors, cost weights, and prediction horizon are explicit design choices. Wind, motor lag, aerodynamic drag, and model-uncertainty parameter sweeps are not included.
5. The MP4 is generated from MATLAB simulation data. It demonstrates the algorithm's simulated behavior and is not a Gazebo screen recording.

Report these limitations accurately in the course submission. Hardware, background workload, and MATLAB version can all affect wall-clock solve times.

## Related Documents

- [English project guide](README_EN.md)
- [Implementation notes](DESIGN_NOTES_EN.md)
- [Gazebo interface](GAZEBO_INTERFACE_EN.md)
- [Chinese version](VALIDATION.md)
