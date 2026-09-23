function session=main_gazebo(c)
%MAIN_GAZEBO MATLAB-side live simulation controller. See GAZEBO_INTERFACE.md.
% Requires external Gazebo model + bridge + per-rotor thrust adapter.
% Physics must be slowed enough for this solver; no real-time claim is made.
setup_project();
if nargin==0, c=project_config(); end
assert(exist('ros2node','file')==2,'ROS Toolbox is required.');
assert(c.ros.transitionSeconds>0 && c.ros.warmupSeconds>=0);
assert(norm(c.ros.initialVelocity)<1e-9,'ROS startup assumes initial hover.');
ctrl=quad.mpc_init(c);
x0=[c.ros.initialPosition;c.ros.initialVelocity;c.ros.initialQuaternion;zeros(3,1)];
e=quad.ekf_init(x0,c,false); io=quad.RosIO(c); guard=onCleanup(@()delete(io));
tag=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
outdir=fullfile(c.outputRoot,['gazebo_' tag]); mkdir(outdir);
records=zeros(0,1+13+15+4+4); % time, estimate, sigma, thrust, solver telemetry
firstTime=NaN; stateTime=NaN; lastImu=NaN; nextControl=0; lastActivity=tic;
heldA=[0;0;-c.g]; heldW=zeros(3,1); late=0; skipped=0;
u=c.hover; io.send(u); finalTime=c.ros.warmupSeconds+c.ros.transitionSeconds+c.duration;
fprintf('Waiting for timestamped IMU, position and /clock. Use a Gazebo-only ROS domain.\n');
try
    while true
        events=io.pop();
        if isempty(events)
            if toc(lastActivity)>c.ros.timeout, error('quad:Timeout','No usable sensor data. Check bridge, /clock and QoS.'); end
            pause(.001); continue;
        end
        lastActivity=tic;
        for k=1:size(events,1)
            ti=events(k,1); kind=events(k,2); data=events(k,3:8)';
            if isnan(stateTime)
                if kind~=1, continue; end
                firstTime=ti; stateTime=ti; lastImu=ti;
                heldA=data(1:3); heldW=data(4:6);
            end
            if ti<stateTime-1e-8, late=late+1; continue; end
            dt=ti-stateTime;
            if dt>c.ros.maxImuGap, error('quad:Gap','Sensor timestamp gap %.3f s; refusing blind propagation.',dt); end
            if dt>1e-10, e=quad.ekf_predict(e,heldA,heldW,dt,c); stateTime=ti; end
            if kind==1
                if ti-lastImu>c.ros.maxImuGap, error('quad:IMUGap','IMU stream has missing samples.'); end
                heldA=data(1:3); heldW=data(4:6); lastImu=ti; e.omega=heldW-e.bg;
            else
                e=quad.ekf_update(e,data(1:3),c);
            end
        end
        elapsed=stateTime-firstTime;
        if elapsed>=finalTime, break; end
        if elapsed>.25 && (isnan(io.latestPosition)||stateTime-io.latestPosition>.2)
            error('quad:GNSSMissing','Position measurements missing or stale.');
        end
        if io.clock<stateTime-.1, error('quad:ClockReset','Simulation clock moved backwards. Restart this run.'); end
        if io.clock-stateTime>c.ros.maxQueueLag
            error('quad:Latency','Sensor/control backlog %.3f simulated seconds. Reduce simulation speed.',io.clock-stateTime);
        end
        if elapsed+1e-8>=nextControl
            skipped=skipped+max(0,floor((elapsed-nextControl)/c.controlDt));
            ref=quad.ros_reference(elapsed+(1:c.mpc.horizon)*c.controlDt,c);
            [u,ctrl,info]=quad.mpc_step(quad.ekf_state(e),elapsed,ctrl,c,ref);
            io.send(u); nextControl=elapsed+c.controlDt;
            records(end+1,:)=[elapsed,quad.ekf_state(e)',sqrt(max(0,diag(e.P)))',u', ...
                info.solveTime,info.exitflag,info.violation,double(info.fallback)]; %#ok<AGROW>
        end
    end
catch ME
    session.config=c; session.records=records; session.truth=io.truth;
    session.firstTime=firstTime; session.error=ME.message; session.outputDir=outdir;
    save(fullfile(outdir,'interrupted_session.mat'),'session'); rethrow(ME);
end
session.config=c; session.records=records; session.truth=io.truth;
session.firstTime=firstTime; session.latePackets=late; session.skippedControlPeriods=skipped;
session.outputDir=outdir; session.gnssRejected=e.gnssRejected;
save(fullfile(outdir,'gazebo_session.mat'),'session');
if c.ros.requireTruth && isempty(io.truth), error('quad:NoTruth','Control completed, but no truth stream for evaluation.'); end
fprintf('Gazebo session saved: %s\nLate packets=%d, skipped periods=%d\n',outdir,late,skipped);
if ~isempty(io.truth), analyze_gazebo(session); end
end
