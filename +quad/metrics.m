function m=metrics(L,c)
ep=L.truth(1:3,:)-L.reference(1:3,:);
m.positionRMSE=sqrt(mean(sum(ep.^2,1))); m.positionMax=max(vecnorm(ep));
m.positionAxisRMSE=sqrt(mean(ep.^2,2));
ee=L.estimate(1:3,:)-L.truth(1:3,:);
m.estimatedPositionRMSE=sqrt(mean(sum(ee.^2,1)));
angles=zeros(3,numel(L.t));
for k=1:numel(L.t), angles(:,k)=quad.Math.euler(L.truth(7:10,k)); end
m.maxAbsRollPitchDeg=max(abs(angles(1:2,:)),[],2)*180/pi;
m.maxAbsBodyRatesDeg=max(abs(L.truth(11:13,:)),[],2)*180/pi;
m.minRotorThrust=min(L.u,[],2); m.maxRotorThrust=max(L.u,[],2);
tol=1e-6;
m.rotorViolationSamples=sum(any(L.u<c.thrustMin-tol|L.u>c.thrustMax+tol,1));
m.attitudeViolationSamples=sum(any(abs(angles(1:2,:))>c.tiltMax+tol,1));
m.rateViolationSamples=sum(any(abs(L.truth(11:13,:))>c.rateMax+tol,1));
s=L.solveTime(isfinite(L.solveTime));
m.solveMean=mean(s); m.solveMax=max(s); ss=sort(s);
m.solveP95=ss(max(1,ceil(.95*numel(ss))));
m.deadlineMisses=sum(s>c.controlDt); m.controlUpdates=numel(s);
m.fallbackCount=sum(L.fallback); m.nonpositiveExitCount=sum(L.exitflag<=0);
m.gnssRejected=sum(L.gnssRejected);
m.twoSigmaCoverage=mean(abs(L.error)<=2*L.sigma,2);
m.finalTime=L.t(end); m.seed=c.seed;
end
