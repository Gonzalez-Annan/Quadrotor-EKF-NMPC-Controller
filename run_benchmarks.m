function results=run_benchmarks(seeds)
% True-state baseline plus EKF cases; can take several minutes per case.
if nargin<1, seeds=[6224 6225 6226]; end
c=project_config(); c.feedback='truth'; c.runName='baseline_truth';
results={run_simulation(c)};
for seed=seeds
    c=project_config(); c.seed=seed; c.runName=sprintf('ekf_seed_%d',seed);
    results{end+1}=run_simulation(c); %#ok<AGROW>
end
rows=cellfun(@(r)[r.config.seed,strcmp(r.config.feedback,'ekf'), ...
    r.metrics.positionRMSE,r.metrics.positionMax,r.metrics.solveMean, ...
    r.metrics.solveMax,r.metrics.fallbackCount,r.metrics.attitudeViolationSamples], ...
    results,'UniformOutput',false);
T=array2table(vertcat(rows{:}),'VariableNames',{'Seed','EKFFeedback','PositionRMSE_m', ...
    'PositionMax_m','SolveMean_s','SolveMax_s','FallbackCount','AttitudeViolationSamples'});
writetable(T,fullfile(c.outputRoot,'benchmark_comparison.csv')); disp(T);
end
