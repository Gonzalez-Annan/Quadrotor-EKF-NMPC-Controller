function report=test_ros_interface()
% Local synthetic messages, isolated test domain/topics; no Gazebo required.
c=project_config(); c.ros.domainID=198; c.ros.nodeName='/ma6224_test_controller';
c.ros.imuTopic='/ma6224_test/imu'; c.ros.positionTopic='/ma6224_test/position';
c.ros.truthTopic='/ma6224_test/truth'; c.ros.commandTopic='/ma6224_test/thrust';
io=quad.RosIO(c); cleanupIO=onCleanup(@()delete(io));
source=ros2node('/ma6224_test_source',198); cleanupNode=onCleanup(@()delete(source));
ip=ros2publisher(source,c.ros.imuTopic,'sensor_msgs/Imu');
pp=ros2publisher(source,c.ros.positionTopic,'geometry_msgs/PointStamped');
tp=ros2publisher(source,c.ros.truthTopic,'nav_msgs/Odometry');
cp=ros2publisher(source,'/clock','rosgraph_msgs/Clock');
sub=ros2subscriber(source,c.ros.commandTopic,'std_msgs/Float64MultiArray');
imu=ros2message(ip); imu.header.stamp.sec=int32(1); imu.linear_acceleration.z=c.g;
imu.angular_velocity.x=.1; imu.angular_velocity.y=.2; imu.angular_velocity.z=.3;
pos=ros2message(pp); pos.header.stamp.sec=int32(1); pos.point.x=2; pos.point.y=3; pos.point.z=4;
truth=ros2message(tp); truth.header.stamp.sec=int32(1); truth.pose.pose.orientation.w=1;
truth.pose.pose.position=pos.point; truth.twist.twist.linear.x=1;
clock=ros2message(cp); clock.clock.sec=int32(2);
for k=1:30
    send(ip,imu); send(pp,pos); send(tp,truth); send(cp,clock); io.send(c.hover);
    pause(.1); drawnow;
    if any(io.sensorQueue(:,2)==1)&&any(io.sensorQueue(:,2)==2)&&~isempty(io.truth)&&~isempty(sub.LatestMessage), break; end
end
assert(~isempty(io.sensorQueue)&&~isempty(io.truth),'ROS callbacks did not receive synthetic messages.');
a=io.sensorQueue(find(io.sensorQueue(:,2)==1,1),3:8)';
assert(norm(a-[0;0;-c.g;.1;-.2;-.3])<1e-10,'IMU frame conversion failed.');
p=io.sensorQueue(find(io.sensorQueue(:,2)==2,1),3:5)';
assert(norm(p-[3;2;-4])<1e-10,'Position frame conversion failed.');
assert(norm(sub.LatestMessage.data-c.hover)<1e-10,'Rotor command payload mismatch.');
assert(norm(io.truth(1,5:7)'-[0;1;0])<1e-10,'Body velocity conversion failed.');
report.passed=true; report.domain=198; report.time=char(datetime('now'));
fprintf('PASS ROS 2 loopback, IMU/position/truth frames and rotor command payload.\n');
end
