%% Merged project: teammate EKF/model/sensors + constrained NMPC
cd(fileparts(mfilename('fullpath')));
setup_project();
check_environment();
testReport=run_merge_tests();
c=project_config(); c.duration=3;
quickResult=main_simulation(c);

%% Full mission (run this section explicitly)
% c=project_config(); c.makeVideo=true;
% fullResult=main_simulation(c);

%% External Gazebo (requires model, bridge and rotor adapter)
% c=project_config(); session=main_gazebo(c);
