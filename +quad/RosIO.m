classdef RosIO < handle
    % ROS 2 transport only. No estimator may read truth from this object.
    properties
        node; imuSub; posSub; truthSub; clockSub; publisher
        sensorQueue=zeros(0,8); truth=zeros(0,14)
        clock=NaN; latestImu=NaN; latestPosition=NaN
        world; body; config; badMessages=0
    end
    methods
        function obj=RosIO(c)
            obj.config=c; [obj.world,obj.body]=quad.frames(c);
            obj.node=ros2node(c.ros.nodeName,c.ros.domainID);
            obj.imuSub=ros2subscriber(obj.node,c.ros.imuTopic,'sensor_msgs/Imu', ...
                @(m)obj.onImu(m),'Reliability','besteffort','Depth',1000);
            obj.posSub=ros2subscriber(obj.node,c.ros.positionTopic,'geometry_msgs/PointStamped', ...
                @(m)obj.onPosition(m),'Reliability','besteffort','Depth',200);
            obj.truthSub=ros2subscriber(obj.node,c.ros.truthTopic,'nav_msgs/Odometry', ...
                @(m)obj.onTruth(m),'Reliability','besteffort','Depth',1000);
            obj.clockSub=ros2subscriber(obj.node,'/clock','rosgraph_msgs/Clock', ...
                @(m)obj.onClock(m),'Reliability','besteffort','Depth',10);
            obj.publisher=ros2publisher(obj.node,c.ros.commandTopic,c.ros.commandType, ...
                'Reliability','reliable','Depth',1);
        end
        function onImu(obj,m)
            t=quad.RosIO.stamp(m.header.stamp);
            a=obj.body*[m.linear_acceleration.x;m.linear_acceleration.y;m.linear_acceleration.z];
            w=obj.body*[m.angular_velocity.x;m.angular_velocity.y;m.angular_velocity.z];
            if ~all(isfinite([t;a;w])), obj.badMessages=obj.badMessages+1; return; end
            obj.sensorQueue(end+1,:)=[t,1,a',w']; obj.latestImu=t;
        end
        function onPosition(obj,m)
            t=quad.RosIO.stamp(m.header.stamp);
            p=obj.world*[m.point.x;m.point.y;m.point.z];
            if ~all(isfinite([t;p])), obj.badMessages=obj.badMessages+1; return; end
            obj.sensorQueue(end+1,:)=[t,2,p',zeros(1,3)]; obj.latestPosition=t;
        end
        function onTruth(obj,m)
            t=quad.RosIO.stamp(m.header.stamp); pp=m.pose.pose; tw=m.twist.twist;
            q=[pp.orientation.w;pp.orientation.x;pp.orientation.y;pp.orientation.z];
            if norm(q)<0.5, obj.badMessages=obj.badMessages+1; return; end
            Rexternal=quad.Math.rot(q);
            p=obj.world*[pp.position.x;pp.position.y;pp.position.z];
            % nav_msgs/Odometry twist must be in child/body frame.
            v=obj.world*Rexternal*[tw.linear.x;tw.linear.y;tw.linear.z];
            q=quad.Math.fromRot(obj.world*Rexternal*obj.body');
            w=obj.body*[tw.angular.x;tw.angular.y;tw.angular.z];
            x=[p;v;q;w];
            if all(isfinite([t;x])), obj.truth(end+1,:)=[t,x']; end
        end
        function onClock(obj,m)
            obj.clock=quad.RosIO.stamp(m.clock);
        end
        function E=pop(obj)
            drawnow;
            if size(obj.sensorQueue,1)>10000, error('quad:Backlog','ROS sensor queue overflow.'); end
            if isnan(obj.latestImu)||isnan(obj.clock), E=zeros(0,8); return; end
            % Small reorder buffer: position and IMU packets arrive separately.
            watermark=min(obj.latestImu,obj.clock)-0.02;
            mask=obj.sensorQueue(:,1)<=watermark;
            E=sortrows(obj.sensorQueue(mask,:),[1 2]); obj.sensorQueue(mask,:)=[];
        end
        function send(obj,u)
            msg=ros2message(obj.publisher); msg.data=double(u(:)); send(obj.publisher,msg);
        end
        function delete(obj)
            % Simulation only: disarm on normal completion or MATLAB error.
            try
                obj.send(zeros(4,1));
            catch
            end
            names={'imuSub','posSub','truthSub','clockSub','publisher','node'};
            for k=1:numel(names)
                try
                    delete(obj.(names{k}));
                catch
                end
            end
        end
    end
    methods (Static)
        function t=stamp(s)
            t=double(s.sec)+1e-9*double(s.nanosec);
        end
    end
end
