# MA6224 Quadrotor: EKF and Constrained NMPC

This MATLAB project combines the team's quadrotor dynamics, trajectory generator, sensor models and error-state EKF with a constrained nonlinear model predictive controller (NMPC). It provides an offline closed-loop simulation, result exports, plotting and optional video generation.

## Requirements

- MATLAB with Optimization Toolbox: the offline controller uses `fmincon`.
- The recorded integration tests used MATLAB R2026a Update 4 on Windows.
- The offline workflow does not use Simulink or `nlmpc`.

## Quick start

Set MATLAB's Current Folder to the folder containing this README, then run:

```matlab
main_simulation
```

Alternatively, open `main_simulation.m` and click **Run**. It is the top-level script and performs project setup, environment checks, startup tests and a 60-second simulated mission. The former `START_HERE.m` workflow is incorporated into this script.

The settings at the top of the script are:

```matlab
simulationDuration = 60;   % Use 3 for a short smoke run.
runStartupTests = true;
makePlots = true;
makeVideo = false;
```

Plots are saved to disk; figure visibility defaults to off. Completion time depends on the computer and solver load. A 60-second simulated mission does not necessarily finish in 60 seconds of wall-clock time.

For custom settings, call the underlying function instead:

```matlab
setup_project;
c = project_config();
c.duration = 3;
c.figureVisible = 'on';
c.makeVideo = false;
result = run_simulation(c);
disp(result.outputDir);
disp(result.metrics);
```

Use `setup_project` rather than recursively adding multiple copies of the project with `addpath(genpath(...))`. Setup checks important function locations for name shadowing. This project is portable and does not require the original Desktop folder.

## How the simulation works

Reference trajectory → NMPC → four rotor thrusts → quadrotor dynamics → IMU/GNSS measurements → EKF → NMPC feedback.

Ground truth generates simulated measurements and evaluation metrics. Normal controller feedback uses the EKF estimate. Set `c.feedback = 'truth'` only for the separate true-state controller baseline.

The internal world frame is NED, the body frame is FRD, and quaternions are scalar-first `[w x y z]'`, rotating body vectors into the world frame. The plant state contains position, velocity, quaternion and body angular velocity. The EKF additionally estimates accelerometer and gyroscope biases.

| Default setting | Value |
|---|---|
| Mass | 1.20 kg |
| Principal inertias | [0.0125, 0.0125, 0.0220] kg·m² |
| Physics / IMU / GNSS / control rates | 500 / 100 / 20 / 20 Hz |
| Mission duration / random seed | 60 s / 6224 |
| NMPC prediction horizon | 20 control steps (1 s) |
| Control blocking | [1 1 2 3 5 8] |
| Per-rotor thrust bounds | 0.2–5.5 N |
| Roll and pitch bounds | ±35° |
| Body-rate bounds | ±[180, 180, 90]°/s |

The offline mission starts on the moving reference, not from a ground takeoff. Parameters originate in `getDefaultParams.m`; `project_config.m` adds controller, estimator, output settings. The configuration supplied to a run is authoritative and is saved with its results. Edit the configuration rather than hard-coding parameters in the dynamics or EKF.

## Project map and authorship

| File or folder | Purpose and origin |
|---|---|
| `main_simulation.m` | Top-level executable script |
| `run_simulation.m` | Integrated offline simulation loop |
| `project_config.m`, `setup_project.m` | Configuration and path setup |
| `getDefaultParams.m`, `quadrotorDynamics.m` | Original teammate defaults and plant dynamics |
| `generateTrajectory.m` | Original teammate trajectory |
| `simulateIMU.m`, `simulateGNSS.m` | Original teammate sensor models |
| `EKF part/` | Original teammate estimator functions and tests |
| `+integration/` | Parameter translation and plant integration adapters |
| `+quad/` | NMPC, prediction model, estimator adapters, plotting, video utilities |
| `ekf.m`, `nmpc.m`, `plotting.m` | Public wrapper functions; see MATLAB help for signatures |
| `run_merge_tests.m` | Numerical and integration tests |
| `validate_merged_project.m` | Full offline validation |
| `validate_merged_interfaces.m` | Offline public wrapper checks |
| `run_benchmarks.m` | True-state baseline and configurable EKF seeds |
| `results/` | Simulation data and figures |
| `validation/` | Recorded test evidence |
| `provenance/` | Original-source archives and hash manifests |

The teammate's nonempty MATLAB source files remain unchanged. Four originally empty MATLAB placeholders were filled during integration. This README replaces the separate project Markdown documents, including the copied teammate README. The original documentation remains available inside `provenance/teammate_original.zip`; the original Desktop source directory is untouched.

The earlier mixed-purpose source archive is no longer distributed with this offline project. Its historical results must not be presented as validation of the merged project. The merged sensor bias settings and measurement timing follow the teammate code, so the same seed need not reproduce the earlier project's samples.

## Outputs

Each offline run creates `results/<runName>/`. With the default empty `c.runName`, the code generates a timestamped name. A fixed run name reuses the corresponding output location.

- `result.mat`: saved simulation result.
- `configuration.json`: effective configuration.
- `metrics.json`: tracking, estimation, constraint and solver metrics.
- `telemetry.csv`: logged time histories.
- Five figure sets in PNG, SVG and FIG formats when plotting is enabled: trajectory, tracking errors, EKF bounds, actuator constraints and solver performance.
- Optional video when `c.makeVideo = true`.

Access the returned data through `result.log`, `result.config`, `result.metrics` and `result.outputDir`. Figures and metrics derive from the simulation logs.

## Validation and current status

The integration validation recorded on 2026-09-23 passed nine inherited numerical tests, four integration test groups, the four original teammate test scripts, a 3-second smoke simulation and two 60-second missions. Public EKF/NMPC wrapper checks also passed.

The offline-only top-level script was successfully executed in MATLAB R2026a Update 4 on 2026-09-24 after interface removal. Seven numerical tests, four integration groups and public wrapper checks passed. The full 60-second mission produced 6001 samples, 1200 successful control updates and all five PNG/SVG/FIG figure sets, which were reopened or checked for readability.

Tracking RMSE was 0.039743 m; estimation RMSE was 0.013452 m. No controller fallbacks or logged constraint violations occurred. Simulation-loop wall time was 103.26 s; mean solver time was 81.34 ms, and 1155 solves exceeded 50 ms. Wall-clock 20 Hz operation is not established.

See [current entry-point validation](validation/entrypoint_runtime.json), [execution log](validation/entrypoint_runtime.log) and [latest result data](results/20260924_112940_127/metrics.json). The table below retains the earlier 2026-09-23 mission metrics.

Recorded 60-second results (seed 6224; 1200 control updates):

| Metric | EKF feedback | True-state baseline |
|---|---:|---:|
| Position tracking RMSE (m) | 0.039743 | 0.008436 |
| Maximum position error (m) | 0.082465 | 0.023369 |
| Position estimation RMSE (m) | 0.013452 | 0.013406 |
| Mean optimization time (ms) | 71.36 | 43.84 |
| Maximum optimization time (ms) | 193.34 | 128.71 |
| Optimizations exceeding 50 ms | 871 | 533 |
| Control fallbacks | 0 | 0 |
| Nonpositive solver exit flags | 0 | 0 |
| Logged thrust / attitude / rate violations | 0 / 0 / 0 | 0 / 0 / 0 |
| Rejected GNSS updates | 2 | 2 |

These results establish offline behavior for the recorded seed. They do not establish 20 Hz wall-clock operation, continuous-time constraint guarantees or broad robustness. Physical flight remains untested. Video generation was not rerun in the merged validation.

To reproduce the checks:

```matlab
setup_project;
testReport = run_merge_tests();
summary = validate_merged_project();
interfaces = validate_merged_interfaces();
```

Full validation takes several minutes and regenerates fixed validation run folders. Original test scripts call `close all`; save figures you need before running them.

Evidence:

- [Offline validation summary](validation/validation_summary.json)
- [MATLAB validation log](validation/matlab_validation.log)
- [Interface validation](validation/interface_validation.json)
- [Entry-point refactor record](validation/entrypoint_refactor.json)
- [Source comparison](provenance/source_comparison.json)
- [Recorded EKF mission](results/merged_ekf_60s/metrics.json)
- [Recorded true-state mission](results/merged_truth_60s/metrics.json)

For additional seeds, run `run_benchmarks([6224 6225 6226])`. This creates new benchmark results and may take several minutes per case.


## Offline-only project update

This distribution contains only the MATLAB offline workflow. The current startup suite has seven numerical tests and four integration groups. The two external-interface-specific numerical tests were removed; the mixer consistency assertion remains part of the physics test. The EKF, plant and NMPC algorithms are unchanged.

Historical mission data and metrics are retained. Unused external-interface configuration fields were removed from saved result configurations; this metadata cleanup does not change state histories, metrics or plots. Earlier validation records describe the version tested at their recorded date.
