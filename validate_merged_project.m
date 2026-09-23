function summary=validate_merged_project()
%VALIDATE_MERGED_PROJECT Reproducible validation, with fresh results only.
root=setup_project(); oldFolder=pwd; cd(root);
folderGuard=onCleanup(@()cd(oldFolder));
out=fullfile(root,'validation'); if ~isfolder(out), mkdir(out); end
diary(fullfile(out,'matlab_validation.log'));
diaryGuard=onCleanup(@()diary('off'));
oldVisible=get(groot,'defaultFigureVisible'); set(groot,'defaultFigureVisible','off');
visibleGuard=onCleanup(@()set(groot,'defaultFigureVisible',oldVisible));
summary.matlab=version; summary.started=char(datetime('now'));
try
    summary.integration=run_merge_tests();
    scripts={'test_quadrotorDynamics.m','test_generate_trajectory.m', ...
        'test_sensors.m',fullfile('EKF part','test_ekf.m')};
    for k=1:numel(scripts)
        [~,name]=fileparts(scripts{k});
        sandbox=fullfile(out,'teammate_tests',name); if ~isfolder(sandbox), mkdir(sandbox); end
        try
            output=executeOriginal(fullfile(root,scripts{k}),sandbox);
            fid=fopen(fullfile(sandbox,'console.txt'),'w'); fprintf(fid,'%s',output); fclose(fid);
            assert(~contains(output,'FAIL:'),'merged:OriginalTestFailed','Original script printed FAIL.');
            summary.original(k).name=scripts{k}; summary.original(k).passed=true;
            fprintf('PASS original %s\n',scripts{k});
        catch ME
            summary.original(k).name=scripts{k}; summary.original(k).passed=false;
            summary.original(k).error=ME.message;
            fprintf('FAIL original %s: %s\n',scripts{k},ME.message);
        end
    end
    cd(root);
    c=project_config(); c.duration=3; c.makePlots=false; c.runName='merged_smoke_3s';
    smoke=main_simulation(c); checkRun(smoke); summary.smoke=smoke.metrics;
    c=project_config(); c.runName='merged_ekf_60s'; c.makePlots=true;
    full=main_simulation(c); checkRun(full); summary.ekf=full.metrics;
    c=project_config(); c.feedback='truth'; c.runName='merged_truth_60s'; c.makePlots=false;
    baseline=main_simulation(c); checkRun(baseline); summary.truth=baseline.metrics;
    files=dir(fullfile(root,'**','*.m')); issues=struct('file',{},'messages',{});
    for k=1:numel(files)
        f=fullfile(files(k).folder,files(k).name);
        if contains(f,[filesep 'validation' filesep]), continue; end
        messages=checkcode(f,'-id');
        if ~isempty(messages), issues(end+1)=struct('file',f,'messages',messages); end %#ok<AGROW>
    end
    summary.codeAnalyzer=issues;
    summary.completed=char(datetime('now'));
    summary.passed=all([summary.original.passed]);
    save(fullfile(out,'validation_summary.mat'),'summary');
    fid=fopen(fullfile(out,'validation_summary.json'),'w');
    fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true)); fclose(fid);
    fprintf('MERGED_VALIDATION_FINISHED original_tests_all_pass=%d\n',summary.passed);
catch ME
    summary.failure=getReport(ME,'extended','hyperlinks','off');
    save(fullfile(out,'validation_failure.mat'),'summary');
    fid=fopen(fullfile(out,'failure.txt'),'w'); fprintf(fid,'%s',summary.failure); fclose(fid);
    fprintf('%s\n',summary.failure);
end
end
function output=executeOriginal(scriptFile,sandbox)
% Script clear/clc commands stay inside this function workspace.
% Evaluate text here so relative figure outputs remain inside validation.
originalCode=fileread(scriptFile);
cd(sandbox);
output=evalc(originalCode);
end
function checkRun(result)
m=result.metrics;
assert(m.positionRMSE<.15,'merged:Tracking','Tracking RMSE exceeds 15 cm.');
assert(m.fallbackCount==0 && m.nonpositiveExitCount==0,'merged:Solver','Solver did not converge normally.');
assert(m.rotorViolationSamples==0 && m.attitudeViolationSamples==0 && m.rateViolationSamples==0);
assert(all(isfinite(result.log.truth(:))) && all(isfinite(result.log.estimate(:))));
assert(max(abs(vecnorm(result.log.truth(7:10,:))-1))<1e-10);
end
