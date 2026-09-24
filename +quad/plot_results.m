function plot_results(r)
L=r.log; c=r.config; d=r.outputDir; t=L.t;
reqDir=fullfile(d,'required'); if ~isfolder(reqDir), mkdir(reqDir); end
extraDir=fullfile(d,'extra'); if ~isfolder(extraDir), mkdir(extraDir); end
f=figure('Visible',c.figureVisible,'Color','w','Position',[80 80 1100 700]);
if isprop(f,'Theme'), f.Theme='light'; end
if isprop(f,'Theme'), f.Theme='light'; end
plot3(L.reference(1,:),L.reference(2,:),-L.reference(3,:),'k--','LineWidth',1.4); hold on;
plot3(L.truth(1,:),L.truth(2,:),-L.truth(3,:),'b','LineWidth',1.2);
plot3(L.estimate(1,:),L.estimate(2,:),-L.estimate(3,:),'Color',[.85 .3 .1]);
grid on; axis equal; xlabel('North (m)'); ylabel('East (m)'); zlabel('Height (m)');
legend('Reference','Truth','EKF','Location','best'); title('3D figure-eight tracking'); view(40,25);
saveplot(f,reqDir,'01_trajectory');
f=figure('Visible',c.figureVisible,'Color','w','Position',[80 80 1100 850]);
if isprop(f,'Theme'), f.Theme='light'; end
if isprop(f,'Theme'), f.Theme='light'; end
labels={'Position error (m)','Velocity error (m/s)','Attitude error (deg)','Body-rate error (deg/s)'};
rotErr=zeros(3,numel(t));
for k=1:numel(t)
    rotErr(:,k)=quad.Math.log(quad.Math.mul(quad.Math.conj(L.reference(7:10,k)),L.truth(7:10,k)))*180/pi;
end
errors={L.truth(1:3,:)-L.reference(1:3,:),L.truth(4:6,:)-L.reference(4:6,:), ...
    rotErr,(L.truth(11:13,:)-L.reference(11:13,:))*180/pi};
tiledlayout(4,1,'TileSpacing','compact');
for j=1:4
    nexttile; plot(t,errors{j}'); grid on; ylabel(labels{j});
    if j<=2, legend('North','East','Down','Location','northeast');
    else, legend('x','y','z','Location','northeast'); end
end
xlabel('Simulation time (s)'); saveplot(f,reqDir,'02_tracking_errors');
f=figure('Visible',c.figureVisible,'Color','w','Position',[40 40 1350 1000]);
if isprop(f,'Theme'), f.Theme='light'; end
if isprop(f,'Theme'), f.Theme='light'; end
tiledlayout(5,3,'TileSpacing','compact','Padding','compact');
units={'p (m)','v (m/s)','attitude (rad)','accel bias (m/s^2)','gyro bias (rad/s)'};
for j=1:15
    nexttile; plot(t,L.error(j,:),'b'); hold on;
    plot(t,2*L.sigma(j,:),'r--',t,-2*L.sigma(j,:),'r--'); grid on;
    title(sprintf('%s / axis %d',units{ceil(j/3)},mod(j-1,3)+1));
    if j>12, xlabel('Time (s)'); end
end
sgtitle('Error-state EKF: blue = truth minus estimate; red = +/-2 sigma'); saveplot(f,reqDir,'03_ekf_bounds');
f=figure('Visible',c.figureVisible,'Color','w','Position',[80 80 1100 900]);
if isprop(f,'Theme'), f.Theme='light'; end
if isprop(f,'Theme'), f.Theme='light'; end
tiledlayout(4,1,'TileSpacing','compact');
nexttile; plot(t,L.u'); hold on; yline(c.thrustMin,'r--'); yline(c.thrustMax,'r--');
ylabel('Rotor thrust (N)'); legend('T0','T1','T2','T3'); grid on;
nexttile; plot(t,(c.mix(2:4,:)*L.u)'); ylabel('Body torque (N m)'); grid on;
legend('Roll','Pitch','Yaw');
angles=zeros(3,numel(t)); for k=1:numel(t), angles(:,k)=quad.Math.euler(L.truth(7:10,k)); end
nexttile; plot(t,angles(1:2,:)'*180/pi); hold on; yline(35,'r--'); yline(-35,'r--');
ylabel('Roll / pitch (deg)'); legend('Roll','Pitch','Location','northeast'); grid on;
nexttile; plot(t,(L.truth(11:13,:)./c.rateMax)'); hold on; yline(1,'r--'); yline(-1,'r--');
ylabel('Rate / limit'); legend('p','q','r','Location','northeast'); xlabel('Simulation time (s)'); grid on;
saveplot(f,reqDir,'04_actuators_constraints');
f=figure('Visible',c.figureVisible,'Color','w','Position',[80 80 1100 600]);
if isprop(f,'Theme'), f.Theme='light'; end
if isprop(f,'Theme'), f.Theme='light'; end
idx=isfinite(L.solveTime); tiledlayout(2,1);
nexttile; plot(t(idx),1000*L.solveTime(idx)); hold on; yline(1000*c.controlDt,'r--','Control period');
ylabel('Solve time (ms)'); grid on;
nexttile; stairs(t(idx),L.exitflag(idx)); hold on; stem(t(L.fallback),L.exitflag(L.fallback),'r');
ylabel('Solver exit flag'); xlabel('Simulation time (s)'); grid on;
saveplot(f,extraDir,'05_solver_performance');
end
function saveplot(f,d,name)
exportgraphics(f,fullfile(d,[name '.png']),'Resolution',160);
print(f,fullfile(d,[name '.svg']),'-dsvg');
savefig(f,fullfile(d,[name '.fig']));
% Only auto-close when running invisibly (batch/professor-safe default).
% When figures are made visible (see RUN_PROJECT.m), leave them open so
% they're actually visible on screen after the run, not saved-then-closed.
if strcmp(get(f,'Visible'),'off'), close(f); end
end
