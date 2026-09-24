%% MAIN_SIMULATION Top-level executable for the merged MA6224 project.
% Run this file with the MATLAB Run button or type: main_simulation
% It incorporates the former START_HERE workflow and runs the full mission.
% For parameterized/batch execution use: result = run_simulation(c).

%% User settings
simulationDuration = 60;   % Set to 3 for a quick example.
runStartupTests = true;
makePlots = true;
makeVideo = true;

%% Project setup and checks
cd(fileparts(mfilename('fullpath')));
setup_project();
environmentStatus = check_environment();
assert(environmentStatus.optimization, 'Optimization Toolbox is required.');
if runStartupTests
    testReport = run_merge_tests();
end

%% Configure and run the mission
c = project_config();
c.duration = simulationDuration;
c.makePlots = makePlots;
c.makeVideo = makeVideo;
result = run_simulation(c);
