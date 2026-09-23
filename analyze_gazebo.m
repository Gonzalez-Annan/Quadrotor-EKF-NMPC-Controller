function result=analyze_gazebo(session)
% Align recorded truth to controller timestamps; exclude startup from scoring.
if ischar(session)||isstring(session), s=load(session,'session'); session=s.session; end
R=session.records; c=session.config; truth=session.truth;
assert(~isempty(R)&&~isempty(truth),'Need both controller records and truth.');
[tt,idx]=unique(truth(:,1),'sorted'); truth=truth(idx,2:14);
for k=2:size(truth,1)
    if dot(truth(k,7:10),truth(k-1,7:10))<0, truth(k,7:10)=-truth(k,7:10); end
end
missionStart=c.ros.warmupSeconds+c.ros.transitionSeconds;
absTime=session.firstTime+R(:,1);
mask=R(:,1)>=missionStart & absTime>=tt(1) & absTime<=tt(end);
R=R(mask,:); absTime=absTime(mask);
assert(size(R,1)>2,'No mission samples overlap with truth.');
L.t=R(:,1)'-missionStart;
L.truth=interp1(tt,truth,absTime,'linear')'; L.truth(7:10,:)=quad.Math.normalize(L.truth(7:10,:));
L.estimate=R(:,2:14)'; L.sigma=R(:,15:29)'; L.u=R(:,30:33)';
L.solveTime=R(:,34)'; L.exitflag=R(:,35)'; L.predictedViolation=R(:,36)'; L.fallback=logical(R(:,37)');
ref=quad.reference(L.t,c); L.reference=ref.x; L.error=nan(15,numel(L.t));
L.error(1:6,:)=L.truth(1:6,:)-L.estimate(1:6,:);
for k=1:numel(L.t)
    L.error(7:9,k)=quad.Math.log(quad.Math.mul(quad.Math.conj(L.estimate(7:10,k)),L.truth(7:10,k)));
end
% Ground-truth IMU biases are not available on the standard ROS interface.
L.gnssRejected=false(size(L.t)); L.bias=nan(6,numel(L.t));
result.log=L; result.config=c; result.outputDir=session.outputDir;
result.metrics=quad.metrics(L,c); result.metrics.twoSigmaCoverage(10:15)=NaN;
result.metrics.gnssRejected=session.gnssRejected;
result.metrics.latePackets=session.latePackets; result.metrics.skippedControlPeriods=session.skippedControlPeriods;
result.metrics.scoredFrom=L.t(1); result.metrics.scoredTo=L.t(end);
save(fullfile(session.outputDir,'result.mat'),'result');
quad.export_results(result); quad.plot_results(result);
end
