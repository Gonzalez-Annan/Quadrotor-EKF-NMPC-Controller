function result=main_simulation(c)
%MAIN_SIMULATION Reproducible 60-s EKF + constrained NMPC benchmark.
%   result = main_simulation();
%   c=project_config; c.duration=3; result=main_simulation(c);
setup_project();
if nargin==0, c=project_config(); end
p=integration.to_params(c);
assert(exist('fmincon','file')==2,'Optimization Toolbox is required.');
assert(any(strcmp(c.feedback,{'ekf','truth'})),'feedback must be ekf or truth.');
assert(abs(c.controlDt/c.dt-round(c.controlDt/c.dt))<1e-9);
assert(abs(c.gnssDt/c.dt-round(c.gnssDt/c.dt))<1e-9);
assert(abs(c.dt/c.physicsDt-round(c.dt/c.physicsDt))<1e-9);
rng(c.seed,'twister');
if isempty(c.runName), c.runName=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')); end
outdir=fullfile(c.outputRoot,c.runName); if ~isfolder(outdir), mkdir(outdir); end
t=0:c.dt:c.duration; N=numel(t); ref=quad.reference(t,c);
% Begin on the moving reference, not on the ground. Startup is separate.
x=ref.x(:,1); e=quad.ekf_init(x,c,true); ctrl=quad.mpc_init(c);
bias=[c.sensor.initialAccelBias;c.sensor.initialGyroBias];
L.t=t; L.truth=nan(13,N); L.estimate=nan(13,N); L.reference=ref.x;
L.u=nan(4,N); L.bias=nan(6,N); L.error=nan(15,N); L.sigma=nan(15,N);
L.positionMeasurement=nan(3,N); L.nis=nan(1,N); L.gnssRejected=false(1,N);
L.solveTime=nan(1,N); L.exitflag=nan(1,N); L.cost=nan(1,N);
L.predictedViolation=nan(1,N); L.fallback=false(1,N); L.iterations=nan(1,N);
u=c.hover; k=1;
fprintf('Run %s: %.2f s, %s feedback, %d prediction steps\n',c.runName,c.duration,c.feedback,c.mpc.horizon);
wall=tic;
try
    for k=1:N
        if mod(k-1,round(c.gnssDt/c.dt))==0
            z=simulateGNSS(x,p);
            oldRejected=e.gnssRejected; e=quad.ekf_update(e,z,c);
            L.positionMeasurement(:,k)=z; L.nis(k)=e.lastNIS;
            L.gnssRejected(k)=e.gnssRejected>oldRejected;
        end
        xhat=quad.ekf_state(e);
        if k<N && mod(k-1,round(c.controlDt/c.dt))==0
            if strcmp(c.feedback,'truth'), feedback=x; else, feedback=xhat; end
            [u,ctrl,info]=quad.mpc_step(feedback,t(k),ctrl,c);
            L.solveTime(k)=info.solveTime; L.exitflag(k)=info.exitflag;
            L.cost(k)=info.cost; L.predictedViolation(k)=info.violation;
            L.fallback(k)=info.fallback; L.iterations(k)=info.iterations;
        end
        L.truth(:,k)=x; L.estimate(:,k)=xhat; L.u(:,k)=u; L.bias(:,k)=bias;
        L.error(:,k)=quad.estimation_error(e,x,bias);
        L.sigma(:,k)=sqrt(max(0,diag(e.P)));
        if k==N, break; end
        mid=integration.plant_step(x,u,c.dt/2,p,ceil(c.dt/(2*c.physicsDt)));
        % Original sensor advances bias BEFORE measurement. Keep its timing.
        [imu,baNext,bgNext]=simulateIMU(mid,quadrotorDynamics(mid,u,p), ...
            bias(1:3),bias(4:6),c.dt,p);
        x=integration.plant_step(x,u,c.dt,p,round(c.dt/c.physicsDt));
        e=quad.ekf_predict(e,imu.accel,imu.gyro,c.dt,c);
        bias=[baNext;bgNext];
        assert(all(isfinite(x))&&all(isfinite(e.P(:))),'Simulation produced nonfinite state.');
        if mod(k,round(5/c.dt))==0
            fprintf('  t=%.1f s, position error=%.3f m, elapsed=%.1f s\n',t(k),norm(x(1:3)-ref.p(:,k)),toc(wall));
            drawnow limitrate;
        end
    end
catch ME
    save(fullfile(outdir,'failed_run.mat'),'L','c','k','ME');
    rethrow(ME);
end
result.log=L; result.config=c; result.outputDir=outdir;
result.metrics=quad.metrics(L,c); result.metrics.totalWallTime=toc(wall);
save(fullfile(outdir,'result.mat'),'result','-v7');
quad.export_results(result);
if c.makePlots, quad.plot_results(result); end
if c.makeVideo, quad.make_video(result); end
disp(result.metrics); fprintf('Saved: %s\n',outdir);
end
