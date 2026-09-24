function c = project_config()
%PROJECT_CONFIG All physical parameters, assumptions, and tuning in one place.
% World NED; body FRD; q=[w x y z]' rotates body vectors to world.
setup_project(); p=getDefaultParams();
c.mass=p.m; c.g=p.g; c.J=diag(p.J);
c.dx=p.dx; c.dy=p.dy; c.ctau=p.ctau;
c.mix=[ones(1,4);c.dy*[-1 -1 1 1];c.dx*[-1 1 1 -1];c.ctau*[-1 1 -1 1]];
c.thrustMin=p.thrust_min_per_rotor; c.thrustMax=p.thrust_max_per_rotor;
c.tiltMax=p.tilt_max_deg*pi/180; c.rateMax=p.rate_max_deg(:)*pi/180;
c.hover=ones(4,1)*c.mass*c.g/4;
c.dt=1/p.imu_rate_hz; c.physicsDt=0.002; c.gnssDt=1/p.gnss_rate_hz; c.controlDt=0.05;
c.duration=p.traj.tf; c.traj=p.traj; c.seed=6224; c.feedback='ekf';
c.sensor.accelSigma=p.sigma_accel; c.sensor.gyroSigma=p.sigma_gyro; c.sensor.positionSigma=p.sigma_pos;
% Bias random-walk densities are design assumptions, not specified by brief.
c.sensor.accelBiasRW=p.sigma_ba_rw; c.sensor.gyroBiasRW=p.sigma_bg_rw;
c.sensor.initialAccelBias=[0.025;-0.02;0.015];
c.sensor.initialGyroBias=[0.002;-0.0015;0.001];
c.ekf.initialPositionError=[0.015;-0.01;0.01];
c.ekf.initialVelocityError=[0.02;-0.01;0.01];
c.ekf.initialAttitudeError=[0.3;-0.3;0.5]*pi/180;
c.ekf.initialStd=[0.05*ones(3,1);0.1*ones(3,1);2*pi/180*ones(3,1); ...
    0.08*ones(3,1);0.008*ones(3,1)];
c.ekf.gnssGate=16.27; % 3-DOF chi-square threshold, approximately 99.9%.
if isfield(p,'ekf')
    fields=fieldnames(p.ekf);
    for k=1:numel(fields), c.ekf.(fields{k})=p.ekf.(fields{k}); end
end
c.integration.version='merged-2026-09-23';
c.integration.estimator='unchanged EKF part functions';
c.integration.plant='unchanged quadrotorDynamics';
c.integration.sensors='unchanged simulateIMU and simulateGNSS';
c.integration.predictor='equivalent vectorized quad.dynamics for complex-step gradients';
c.mpc.horizon=20; c.mpc.blocks=[1 1 2 3 5 8];
c.mpc.inputMap=c.mix\diag([c.mass;c.J]); % Virtual acceleration -> rotor increments.
c.mpc.maxIterations=40; c.mpc.constraintTolerance=1e-5;
c.mpc.acceptTolerance=1e-4; c.mpc.optimalityTolerance=0.03;
c.mpc.positionWeight=[30;30;45]; c.mpc.velocityWeight=[5;5;8];
c.mpc.attitudeWeight=12; c.mpc.rateWeight=[0.15;0.15;0.25];
c.mpc.inputWeight=0.03; c.mpc.slewWeight=0.15; c.mpc.terminalMultiplier=4;
c.mpc.integrationSubsteps=2; c.mpc.maxConsecutiveFailures=5;
c.makePlots=true; c.makeVideo=true; c.figureVisible='on';
c.outputRoot=fullfile(fileparts(mfilename('fullpath')),'results');
c.runName='';
% ROS interface: configuration must match the Gazebo bridge/adapter.
c.ros.domainID=0; c.ros.nodeName='/matlab_quadrotor';
c.ros.imuTopic='/quad/imu'; c.ros.positionTopic='/quad/position';
c.ros.truthTopic='/quad/truth'; c.ros.commandTopic='/quad/thrust_cmd';
c.ros.timeout=15; c.ros.maxImuGap=0.035; c.ros.maxQueueLag=0.15;
c.ros.worldFrame='ENU'; c.ros.bodyFrame='FLU';
c.ros.initialPosition=[0;0;-2]; c.ros.initialVelocity=zeros(3,1);
c.ros.initialQuaternion=quad.Math.fromEuler(0,0,atan2(1,0.75));
c.ros.warmupSeconds=3; c.ros.transitionSeconds=4;
c.ros.requireTruth=false;
c.ros.commandType='std_msgs/Float64MultiArray';
% The external adapter MUST map data=[T0 T1 T2 T3] N into rotor forces.
% This topic is NOT directly a Gazebo motor-speed command.
end
