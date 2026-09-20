# MA6224: Quadrotor EKF and NMPC in MATLAB

This project implements a nonlinear quadrotor model, an error-state Extended Kalman Filter (EKF), and a constrained Nonlinear Model Predictive Controller (NMPC) for tracking the prescribed three-dimensional figure-eight trajectory.

The project includes standalone MATLAB simulation, MATLAB-side ROS 2 interfaces for Gazebo, numerical tests, plots, flight visualization, and saved experiment results. External Gazebo integration has not yet been validated. See [VALIDATION_EN.md](VALIDATION_EN.md) for the recorded validation scope and limitations.

## Quick Start

Set the folder containing this file as the MATLAB **Current Folder**, then run:

```matlab
check_environment
run_tests

% Start with a short simulation to check execution speed and plotting.
c = project_config;
c.duration = 3;
quick = main_simulation(c);

% Run the prescribed 60-second mission using estimated-state feedback.
c = project_config;
result = main_simulation(c);

% Generate a video from the saved simulation data without rerunning NMPC.
quad.make_video(result);
```

You can also open `START_HERE.m` and execute its sections individually.

The standalone simulation does not require Gazebo. Before running `main_gazebo`, read [GAZEBO_INTERFACE_EN.md](GAZEBO_INTERFACE_EN.md) and configure the simulator and ROS network accordingly.

## Requirements

- MATLAB
- Optimization Toolbox
- ROS Toolbox for the Gazebo interface

The NMPC implementation uses the SQP algorithm in `fmincon` directly. It does not call `nlmpc`, so Model Predictive Control Toolbox is not a runtime dependency. The nonlinear controller computes the rotor commands itself; it does not rely on an existing flight controller to perform trajectory tracking.

The included experiments were run in MATLAB R2026a Update 4. Other releases have not been tested.

## Project Files

| File | Purpose |
|---|---|
| `project_config.m` | Physical parameters, sensor noise, controller weights, sampling periods, and ROS settings |
| `main_simulation.m` | Standalone MATLAB closed-loop simulation |
| `run_tests.m` | Numerical checks for physics, rotations, trajectory derivatives, optimization gradients, EKF, constraints, and hover control |
| `run_benchmarks.m` | Ground-truth feedback baseline and experiments with different noise seeds |
| `main_gazebo.m` | Live ROS 2 sensor processing, state estimation, and control |
| `analyze_gazebo.m` | Alignment of Gazebo ground truth with MATLAB logs and generation of evaluation plots |
| `test_ros_interface.m` | Local ROS 2 loopback test using synthetic messages |
| `+quad/dynamics.m`, `+quad/step.m` | 13-state rigid-body dynamics and RK4 integration |
| `+quad/reference.m` | Prescribed figure-eight position, velocity, acceleration, and attitude references |
| `+quad/ekf_*.m` | Multiplicative error-state EKF |
| `+quad/mpc_*.m` | Direct-shooting NMPC, move blocking, derivatives, optimization, and failure logging |
| `+quad/Math.m`, `+quad/frames.m` | Rotation and coordinate transformations without additional toolbox dependencies |
| `+quad/plot_results.m`, `+quad/make_video.m` | Evaluation plots and MP4 visualization |
| `+quad/RosIO.m`, `+quad/ros_reference.m` | ROS 2 transport, buffering, and startup reference trajectory |

## Outputs

Each simulation writes to `results/<run_identifier>/`. The default identifier includes a timestamp. Explicitly reusing `c.runName` overwrites files with matching names in that run directory; do this only when intentionally replacing an experiment.

| Output | Contents |
|---|---|
| `result.mat` | Trajectories, estimates, diagonal entries of the 15-dimensional error-state covariance, true sensor biases, control inputs, solver status, and configuration |
| `configuration.json` | Run configuration |
| `metrics.json` | Position RMSE, maximum error, constraint violation sample counts, computation times, and failure counts |
| `telemetry.csv` | Position, velocity, rotor thrust, and solver telemetry |
| `01_trajectory.*` | Three-dimensional reference, true, and estimated trajectories |
| `02_tracking_errors.*` | Position, velocity, attitude, and body-rate tracking errors |
| `03_ekf_bounds.*` | Error-state estimation errors with ±2σ bounds |
| `04_actuators_constraints.*` | Rotor thrusts, body torques, roll/pitch limits, and body-rate limits |
| `05_solver_performance.*` | Wall-clock solve times and solver exit flags |
| `flight_visualization.mp4` | Optional MATLAB animation with flight motion and time-varying error telemetry |

Plots are exported as PNG, SVG, and editable MATLAB FIG files. The MP4 is a visualization generated from MATLAB simulation data, not a Gazebo screen recording.

If the standalone simulation fails during execution, it saves `failed_run.mat` with the data collected so far. Unfilled entries remain NaN and must not be treated as a completed experiment.

To load and inspect a completed run:

```matlab
s = load(fullfile('results', 'validated_60s', 'result.mat'));
result = s.result;
disp(result.metrics)
quad.plot_results(result);
```

## Coordinate Frames and State Conventions

- **World frame:** NED — North, East, Down.
- **Body frame:** FRD — Forward, Right, Down.
- **State order:** `[p(3); v(3); q(4); omega(3)]`.
- **Quaternion convention:** Hamilton, scalar first, `[w; x; y; z]`, representing an active body-to-world rotation.
- **Control input:** `[T0; T1; T2; T3]` in newtons, with rotor numbering consistent with the mixing matrix in the assignment.
- **Thrust direction:** Negative body z-axis.
- **Gravity in NED:** `[0; 0; 9.81]` m/s².
- **Per-rotor thrust bounds:** 0.2–5.5 N.
- **Roll and pitch bounds:** Each independently limited to ±35°.
- **Body-rate bounds:** `[180; 180; 90]`°/s for the three axes.
- All internal angular quantities use radians unless explicitly stated otherwise. Plots convert units as labeled.

The assignment's printed acceleration equation omits an explicit mass factor, and its thrust direction and state ordering are not fully consistent. This implementation uses the physically consistent conventions above, verified through hover equilibrium and mixer-direction tests. These choices should be explained in the final report.

## Sensor Models and EKF

The IMU operates at 100 Hz and the position sensor at 20 Hz. Measurement standard deviations are:

| Measurement | Standard deviation |
|---|---:|
| Accelerometer | 0.08 m/s² |
| Gyroscope | 0.015 rad/s |
| Position | 0.02 m |

Measurement noise is added once. Accelerometer measurements represent body-frame specific force.

The EKF nominal state is `[p; v; q; ba; bg]`, containing 16 stored components. Its covariance is defined for the 15-dimensional error state `[dp; dv; dtheta; dba; dbg]`. A unit quaternion has only three independent attitude-error degrees of freedom. The body-rate estimate is obtained by subtracting the estimated gyroscope bias from the gyroscope measurement; it is combined with the other estimates to form the 13-state controller input.

IMU measurements drive prediction, while GNSS position measurements drive correction. The filter uses the Joseph covariance update, multiplicative attitude-error injection, and covariance reset. Attitude ±2σ plots refer to the local rotation-vector error. The covariance does not contain a separate angular-rate state, so no independent EKF angular-rate ±2σ curve is claimed.

Initial sensor biases and bias random-walk strengths are explicit modeling assumptions because the assignment does not specify them. They are defined in `project_config.m`. IMU noise values are interpreted as per-sample standard deviations; bias random-walk parameters are interpreted per square root of time and discretized accordingly.

Position updates include an innovation gate, and rejected updates are counted. IMU and position measurements do not make yaw and all bias components fully observable under every motion. The experiments use a prior near a known initial attitude and do not claim arbitrary-heading global initialization.

## NMPC Implementation

- Control frequency: 20 Hz.
- Default prediction horizon: 20 steps, corresponding to 1 second.
- Six move blocks: `[1 1 2 3 5 8]`, giving 24 optimization variables.
- Two RK4 substeps per prediction interval, with attitude and body-rate constraints checked after each substep.
- An invertible acceleration-scale transformation improves optimization conditioning. Physical rotor thrust bounds enter the optimization as linear inequalities. The total thrust bounds follow automatically from the individual rotor bounds.
- The cost penalizes position, velocity, attitude, and body-rate errors, deviations from hover thrust, and changes in thrust. Terminal state-error weights are increased.
- The quaternion cost is invariant to quaternion sign. Reference attitude is constructed from reference acceleration and yaw.
- Vectorized complex-step derivatives provide objective and constraint gradients, independently checked against central finite differences.
- The previous predicted input sequence is shifted to initialize the next solve.

An `exitflag` of zero indicates that an iteration or evaluation limit was reached; it does not establish convergence. A candidate is used only if its values are finite, its constraints satisfy the acceptance tolerance, and its exit flag is nonnegative. The exit flag remains in the log. If a candidate is unusable, the controller briefly holds the previous input; repeated failures stop the simulation and preserve the data. Holding the previous input is not a safety guarantee.

Nominal NMPC constraints do not guarantee that the actual noisy system remains within its limits under all model errors. Constraint metrics therefore inspect simulated ground truth. No additional independent torque bounds are imposed, since the assignment specifies torque through the rotor mixing relationship rather than separate torque limits. The plots report the resulting body torques.

## Initial Conditions and Experimental Comparisons

The standard standalone simulation initializes position, velocity, attitude, and body rates at the reference state for `t = 0`. The estimator receives explicitly configured small initial errors. Since the reference has nonzero initial velocity, this is not a takeoff-from-ground experiment. After initialization, the EKF does not read ground truth.

The Gazebo interface is configured to start from stationary hover in the air. A preparation phase is followed by a quintic transition to the prescribed trajectory's initial position, velocity, and acceleration. The preparation and transition phases are excluded from mission evaluation; the prescribed mission then runs for 60 seconds. The Gazebo model must be spawned at the configured initial position and heading.

To compare ground-truth feedback with estimated-state feedback and vary the noise seed:

```matlab
results = run_benchmarks([6224 6225 6226]);
```

Ground-truth feedback is a diagnostic baseline. The final estimator-controller experiment uses `c.feedback = 'ekf'`. Different noise seeds assess repeatability; they are an additional validation choice rather than an explicitly specified grading requirement.

## Included Validation Results

The following 60-second runs are included in `results/`:

| Run | Position RMSE | Maximum position error |
|---|---:|---:|
| Ground-truth feedback baseline | 0.87 cm | 2.26 cm |
| EKF feedback, seed 6224 | 3.87 cm | 9.70 cm |
| EKF feedback, seed 6225 | 3.47 cm | 7.16 cm |

All three runs completed 1,200 control updates with normal solver exits and no recorded rotor-thrust, roll/pitch, or body-rate constraint violations. These checks apply to recorded samples, not a proof of continuous-time constraint satisfaction.

Nine numerical self-tests passed. MATLAB Code Analyzer reported no issues in the 29 MATLAB files checked. A local ROS 2 loopback test verified message transfer and coordinate conversion. Synthetic logs were used to verify Gazebo log alignment and postprocessing. These tests do not establish that the external Gazebo model has been integrated successfully.

See the per-run `metrics.json` files and [VALIDATION_EN.md](VALIDATION_EN.md) for details, including covariance coverage, rejected position updates, and timing limitations.

## Computation Time and Gazebo Synchronization

Solver time is measured using `tic` and `toc` as wall-clock time. The control period is 50 ms. Offline simulation can continue when a solve exceeds 50 ms, but this does not demonstrate real-time operation. Simulation time and wall-clock time must be reported separately.

For the included EKF run with seed 6224, the average solve time was approximately 70 ms, and 961 of 1,200 solves exceeded the 50 ms period. The implementation has therefore not demonstrated sustained real-time control at 20 Hz.

Gazebo must run slowly enough for MATLAB to process its data. Begin with a low real-time factor, then adjust it using the worst observed solve time and communication overhead. ROS bridging alone does not provide lockstep synchronization. The live MATLAB interface monitors message backlog, missing sensor data, and clock rollback, and stops when configured limits are exceeded.

External Gazebo integration remains to be completed. The required simulator model, sensor configuration, bridge, rotor-thrust adapter, and topic/frame conventions are described in [GAZEBO_INTERFACE_EN.md](GAZEBO_INTERFACE_EN.md).

## Further Documentation and Reporting

- [DESIGN_NOTES_EN.md](DESIGN_NOTES_EN.md): Model equations, EKF formulation, NMPC parameterization, and numerical methods.
- [VALIDATION_EN.md](VALIDATION_EN.md): Completed experiments and known limitations.
- [GAZEBO_INTERFACE_EN.md](GAZEBO_INTERFACE_EN.md): External simulator interface requirements.
- [README.md](README.md): Chinese project guide.

English versions of all supporting documents are available through the links above. The original Chinese documents are retained alongside them.

The final course report should explain the selected parameters, implementation choices, observed results, and limitations according to the course rubric. Only configurations actually executed should be presented as experimental results.

## Official References

- [fmincon and gradient interfaces](https://www.mathworks.com/help/optim/ug/fmincon.html)
- [ROS 2 subscribers and callback functions](https://www.mathworks.com/help/ros/ref/ros2subscriber.html)
- [ROS Toolbox system requirements](https://www.mathworks.com/help/ros/gs/ros-system-requirements.html)
- [Gazebo and ROS installation compatibility](https://gazebosim.org/docs/harmonic/ros_installation/)
