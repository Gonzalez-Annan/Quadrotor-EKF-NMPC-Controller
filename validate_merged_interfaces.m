function report=validate_merged_interfaces()
%VALIDATE_MERGED_INTERFACES Optional ROS loopback plus public API checks.
root=setup_project(); report.matlab=version; report.time=char(datetime('now'));
c=project_config(); p=integration.to_params(c); r=quad.reference(0,c);
e=ekf('init',r.x,p,true); e=ekf('predict',e,[0;0;-c.g],zeros(3,1),c.dt,p);
e=ekf('update',e,r.p,p); x=ekf('state',e);
assert(numel(x)==13 && all(isfinite(x)));
ctrl=quad.mpc_init(c); [u,~,info]=nmpc(r.x,0,ctrl,c);
assert(~info.fallback && all(u>=c.thrustMin & u<=c.thrustMax));
report.publicAPIs=true;
report.ros=test_ros_interface();
files=dir(fullfile(root,'**','*.m')); report.codeAnalyzer=struct('file',{},'messages',{});
for k=1:numel(files)
    f=fullfile(files(k).folder,files(k).name);
    if contains(f,[filesep 'validation' filesep]), continue; end
    messages=checkcode(f,'-id');
    if ~isempty(messages), report.codeAnalyzer(end+1)=struct('file',f,'messages',messages); end %#ok<AGROW>
end
fid=fopen(fullfile(root,'validation','interface_validation.json'),'w');
fprintf(fid,'%s',jsonencode(report,PrettyPrint=true)); fclose(fid);
fprintf('MERGED_INTERFACE_VALIDATION_FINISHED\n');
end
