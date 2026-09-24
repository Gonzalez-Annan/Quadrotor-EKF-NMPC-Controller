# MA6224 Quadrotor: EKF State Estimation + NMPC Trajectory Tracking

Nonlinear state estimation (Extended Kalman Filter) and constrained
trajectory tracking (Nonlinear MPC) for a 6-DOF quadrotor tracking a 3D
lemniscate, for MA 6224: Unmanned Vehicles.

## Quick Start

Set this folder as MATLAB's **Current Folder**, then:

```matlab
main_simulation
```

This is the assignment's named entry point, and by default it runs the
full 60 s mission, **pops up all five plots on screen, and saves
everything** (plots, video, metrics) to `results/<timestamp>/`.

A convenience wrapper, `RUN_PROJECT.m`, does the same thing with an
upfront toolbox check and a clean one-page summary printed at the end —
use either one.

## System Requirements

- MATLAB (developed and tested on R2026a)
- **Optimization Toolbox** (required — the NMPC calls `fmincon` directly)
- ROS Toolbox and MPC Toolbox are **not required**. They are only used by
  the optional Gazebo interface (`main_gazebo.m`, `analyze_gazebo.m`,
  `docs/GAZEBO_INTERFACE*.md`), which the standalone path never touches.

## Expected Runtime

Measured on two independent machines: the full 60 s mission takes
**90–105 seconds of wall-clock time**, plus roughly another minute to
render the flight video. **This does not mean the mission runs faster
than real time** — the NMPC's own solve time (mean 70–80 ms) exceeds its
50 ms control period, a known limitation described below. Total time to
run `main_simulation` end to end: budget **2–3 minutes**.

## What Gets Saved, and Where

Every run creates `results/<timestamp>/`, containing:

```
results/<timestamp>/
  result.mat                          <- full session (log, config, metrics)
  required/                            <- the assignment's required plots + video
    01_trajectory.png / .svg / .fig
    02_tracking_errors.png / .svg / .fig
    03_ekf_bounds.png / .svg / .fig
    04_actuators_constraints.png / .svg / .fig
    flight_visualization.avi
  extra/                                <- supporting evidence, not separately required
    05_solver_performance.png / .svg / .fig
    metrics.json
    configuration.json
    telemetry.csv
```

| File (in `required/`) | Assignment requirement |
|---|---|
| `01_trajectory.*` | 3D trajectory tracking vs. reference |
| `02_tracking_errors.*` | Position/velocity/attitude/body-rate error history |
| `03_ekf_bounds.*` | EKF estimation error per state, with ±2σ bounds |
| `04_actuators_constraints.*` | Rotor thrust and body torque vs. constraint boundaries |
| `flight_visualization.avi` | Flight visualization video (max 2 minutes) |

`extra/05_solver_performance.*` isn't on the assignment's named plot
list, but directly supports the required "critical discussion of solver
computation times and execution limits" (see Known Limitations).
`metrics.json`/`configuration.json`/`telemetry.csv` are the same run's
numbers in machine-readable form, useful for the report's tables.

**Video format:** `.avi` (Motion JPEG), not `.mp4`. This is deliberate —
MATLAB's MPEG-4 writer only works on Windows/Mac, and Motion JPEG AVI
plays identically on every platform, so the video isn't tied to whatever
OS the grading machine happens to run.

## Repository Structure and What Each Piece Does

```
main_simulation.m         <- REQUIRED ENTRY POINT. Runs the closed loop:
                              EKF predict/update, NMPC solve, dynamics
                              propagation, logging, plotting, video.
RUN_PROJECT.m              <- convenience wrapper: checks Optimization
                              Toolbox up front, calls main_simulation,
                              prints a clean one-page summary
project_config.m            <- every physical/sensor/NMPC/ROS parameter,
                              in one place (the "c" struct everything else reads)
setup_project.m             <- bootstraps the MATLAB path and verifies every
                              function resolves INSIDE this project, not a
                              same-named file elsewhere on the path.
                              Called automatically by main_simulation.
check_environment.m         <- reports whether Optimization/ROS/MPC
                              Toolboxes are actually usable
run_tests.m                  <- 9 fast numerical self-tests (physics,
                              rotations, trajectory derivatives, NMPC
                              gradients, EKF covariance, hover recovery,
                              frame conversions, constraint boundaries)
run_benchmarks.m             <- runs ground-truth-feedback vs. EKF-feedback
                              across several random seeds, tabulates the comparison
START_HERE.m                 <- a guided script; run it section-by-section
                              in the MATLAB editor for a walkthrough

--- Core package (+quad/) ---
+quad/dynamics.m, step.m     <- vectorized 13-state rigid-body model + RK4
+quad/reference.m            <- trajectory + differential-flatness attitude feedforward
+quad/Math.m                 <- quaternion/rotation utilities (no toolbox dependency)
+quad/state_constraints.m    <- tilt/rate limit inequalities used by the NMPC
+quad/mpc_init.m, mpc_step.m,
+quad/mpc_problem.m          <- NMPC: move-blocked fmincon/SQP with complex-step
                              gradients, warm-starting, and independent
                              post-solve constraint re-checking
+quad/ekf_init.m, ekf_predict.m,
+quad/ekf_update.m, ekf_state.m,
+quad/estimation_error.m     <- thin wrappers that delegate to the actual,
                              independently-authored EKF in "EKF part/"
                              (see note below on that folder name)
+quad/plot_results.m,
+quad/metrics.m,
+quad/export_results.m       <- generates and saves the required/extra plots and data
+quad/make_video.m           <- renders flight_visualization.avi
+quad/print_summary.m        <- the clean run summary RUN_PROJECT.m prints
+quad/RosIO.m, ros_reference.m <- Gazebo-only; unused by the standalone path

+integration/to_params.m     <- translates project_config's "c" struct into
                              the parameter struct the flat validated files
                              (below) expect, so both coding styles can
                              share one config without either being modified
+integration/plant_step.m    <- used by run_merge_tests.m to cross-check
                              the two dynamics implementations agree

--- Independently authored & tested, unmodified by the merge ---
getDefaultParams.m, quadrotorDynamics.m, generateTrajectory.m,
simulateIMU.m, simulateGNSS.m, quat2rotmCustom.m, quatMultiplyCustom.m,
quat2eulerZYX.m, normpdfCustom.m
test_quadrotorDynamics.m, test_generate_trajectory.m, test_sensors.m
  -- each independently validated: hover/free-fall/roll-isolation tests,
     trajectory shape/period/derivative/yaw-continuity tests, sensor
     noise-statistics and bias-random-walk tests. See each test file's
     own comments for what it checks and why.

EKF part/                    <- the actual EKF implementation (error-state,
                              right-multiplicative attitude perturbation,
                              Joseph-form covariance update). Independently
                              tested (test_ekf.m) and later cross-validated
                              with a 30-trial Monte Carlo consistency check
                              (see "Known Limitations" below).
                              NOTE: the space in this folder's name is a
                              known quirk, not a bug -- setup_project.m's
                              addpath call handles it correctly. Renaming
                              it is a cosmetic cleanup, not required for
                              correctness.

--- Optional, not required for grading ---
main_gazebo.m, analyze_gazebo.m,
docs/GAZEBO_INTERFACE.md,
docs/GAZEBO_INTERFACE_EN.md   <- external Gazebo/ROS 2 interface. Requires
                              ROS Toolbox. NOT validated against a real
                              Gazebo instance -- read the doc's own
                              "Known Limitations" before attempting to use it.
test_ros_interface.m          <- local ROS 2 loopback test (no external
                              Gazebo needed), also requires ROS Toolbox

run_merge_tests.m,
validate_merged_interfaces.m,
validate_merged_project.m     <- cross-validation tooling confirming the
                              +quad/ wrappers produce numerically identical
                              results to the original flat files (dynamics
                              to 1e-10, EKF state+covariance to 1e-12) and
                              that the original test scripts still pass
                              unmodified. Not needed to run or grade the
                              project; kept for transparency.
provenance/                   <- an automated integrity manifest generated
                              during the merge. One claim in it (that
                              README.md was untouched) does not match git
                              history; treat this folder as informative,
                              not authoritative -- verify any specific
                              claim independently before citing it.
validation/                   <- output of validate_merged_project.m;
                              regenerate by running that script, not
                              required to be present for grading

ekf.m, nmpc.m, plotting.m     <- one-line compatibility wrappers (e.g.
                              nmpc(x,t,ctrl,c) calls quad.mpc_step(...))
                              for the assignment's originally-named,
                              now-filled entry files
```

Loose `test1_*.png` through `test4_*.png` files in the root are
regenerated evidence from re-running the individual test scripts above
(e.g. `test3_roll_response.png` shows the roll-torque isolation check).
They are not required outputs and can be safely deleted or regenerated
at any time by re-running the corresponding `test_*.m` script.

## Coordinate Frames and State Convention

- **World frame:** NED (North-East-Down). Altitude above ground is
  **negative** z (e.g. the reference trajectory's `z0 = -2.5 m` means
  2.5 m up).
- **Body frame:** FRD (Forward-Right-Down).
- **State:** `x = [p(3); v(3); q(4); omega(3)]`, 13-dimensional.
- **Quaternion:** Hamilton convention, scalar-first `[w;x;y;z]`, active
  body-to-world rotation.
- **Control input:** `u = [T0;T1;T2;T3]` newtons, rotor numbering fixed
  by the mixing matrix in `project_config.m`.

## Known Limitations (report these honestly)

1. **Sustained 20 Hz real-time control has not been demonstrated.** Mean
   NMPC solve time (70-80 ms) exceeds the 50 ms control period; roughly
   1000-1150 of 1200 control updates miss the deadline across a 60 s run
   (varies by machine — confirmed across two independent runs). This does
   not affect offline correctness or reproducibility, only live/hardware
   timing.
2. **Gyro bias and z-axis accelerometer bias are the weakest-observed
   EKF states.** `required/03_ekf_bounds.*`'s corresponding panels show
   the true error occasionally touching or exceeding the ±2σ bound, most
   visibly around the heading/yaw axis. This matches known INS/GNSS
   behavior — these states are only weakly, indirectly observable
   without more aggressive maneuvering — and was independently confirmed
   via a 30-trial Monte Carlo consistency check on the EKF alone (not
   a single noisy run).
3. **IMU/GNSS bias random-walk rates are an explicit assumption.**
   Section 4 of the assignment specifies bias exists but not its drift
   rate; the values used are documented directly in `project_config.m`'s
   comments.
4. **Reproducibility is confirmed, not assumed.** With the fixed seed
   (`c.seed = 6224`), all physics/estimation/control outputs are
   bit-identical across repeated runs — confirmed across multiple runs on
   two different machines. Only wall-clock timing fields (`solveMean`,
   `solveMax`, `deadlineMisses`, `totalWallTime`) vary between runs,
   since they measure real CPU time rather than simulation state.

## Reproducing or Extending This Project

```matlab
check_environment           % confirms Optimization Toolbox is available
run_tests                   % 9 fast numerical self-tests
c = project_config; c.duration = 3; quick = main_simulation(c);  % fast sanity check
main_simulation              % the full 60 s run
```

Optional, not required for grading:

```matlab
run_merge_tests              % cross-validates the merged code against the
                              % original independently-written files
validate_merged_project      % re-runs original test scripts + full
                              % simulations end-to-end, saves a validation report
run_benchmarks                % ground-truth vs. EKF feedback across several seeds
```
