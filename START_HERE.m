%% MA6224 quadrotor project: run this file section by section.
% Set this folder as MATLAB's Current Folder before running.
check_environment;

%% Verify physics, estimator and optimizer.
run_tests;

%% Short independent simulation (no Gazebo required).
c = project_config;
c.duration = 3;
quickResult = main_simulation(c);

%% Full required 60-second mission.
% c = project_config;
% result = main_simulation(c);

%% Export video from an existing result.
% quad.make_video(result);

%% Gazebo integration: configure the external simulator first.
% Read GAZEBO_INTERFACE.md before enabling these lines.
% c = project_config;
% c.ros.requireTruth = true;
% session = main_gazebo(c);
