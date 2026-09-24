function export_results(r)
L=r.log; d=r.outputDir;
extraDir=fullfile(d,'extra'); if ~isfolder(extraDir), mkdir(extraDir); end
d=extraDir;
f=fopen(fullfile(d,'metrics.json'),'w','n','UTF-8');
guard=onCleanup(@()fclose(f)); fprintf(f,'%s',jsonencode(r.metrics,'PrettyPrint',true)); clear guard;
f=fopen(fullfile(d,'configuration.json'),'w','n','UTF-8');
guard=onCleanup(@()fclose(f)); fprintf(f,'%s',jsonencode(r.config,'PrettyPrint',true)); clear guard;
M=[L.t;L.reference(1:3,:);L.truth(1:6,:);L.estimate(1:6,:);L.u; ...
    L.solveTime;L.exitflag;double(L.fallback)]';
names={'time_s','ref_N','ref_E','ref_D','true_N','true_E','true_D', ...
    'true_vN','true_vE','true_vD','est_N','est_E','est_D','est_vN','est_vE','est_vD', ...
    'T0_N','T1_N','T2_N','T3_N','solve_s','exitflag','fallback'};
writetable(array2table(M,'VariableNames',names),fullfile(d,'telemetry.csv'));
end
