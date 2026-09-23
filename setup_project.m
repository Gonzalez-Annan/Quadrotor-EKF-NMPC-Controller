function root=setup_project()
%SETUP_PROJECT Select only this project and its unchanged teammate EKF.
% Do not use addpath(genpath(...)): archived/other projects may shadow files.
root=fileparts(mfilename('fullpath'));
addpath(root,'-begin');
addpath(fullfile(root,'EKF part'),'-begin');
names={'getDefaultParams','quadrotorDynamics','generateTrajectory', ...
    'simulateIMU','simulateGNSS','ekf_init','ekf_predict','ekf_update', ...
    'ekf_state','estimation_error','quad.mpc_step','quad.ekf_init'};
for k=1:numel(names)
    resolved=which(names{k});
    assert(startsWith(lower(resolved),lower([root filesep])), ...
        'merged:PathConflict','%s resolves outside the merged project: %s. Change Current Folder to %s.', ...
        names{k},resolved,root);
end
end
