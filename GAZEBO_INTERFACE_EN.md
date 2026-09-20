# Gazebo–MATLAB Interface Contract

The MATLAB-side implementation is provided. The Gazebo world, SDF model, sensors, ROS bridge, and rotor actuator adapter must be prepared on the Ubuntu/Gazebo side. External Gazebo integration has not yet been validated.

## Versions and Prerequisites

The project's suggested configuration is MATLAB R2026a, ROS 2 Jazzy, Gazebo Harmonic, and Ubuntu 24.04. MATLAB may run on Windows. The network must support bidirectional DDS discovery and data communication. Both sides must use the same `ROS_DOMAIN_ID`, and `ROS_LOCALHOST_ONLY` must not prevent communication between machines. Configure DDS for the actual network environment; the scripts do not modify firewall settings.

1. Create a quadrotor with a mass of 1.20 kg and principal moments of inertia `[0.0125 0.0125 0.0220]` kg·m².
2. Spawn it at the initial ENU position `[0,0,2]` m, with the attitude corresponding to `c.ros.initialQuaternion`.
3. Match rotor numbering, rotation directions, and reaction torques to `c.mix`. Do not assume an existing model uses the same numbering.
4. Configure an IMU and a 20 Hz local position measurement. Add the specified noise and biases once.
5. Publish `/clock`. All measurement `header.stamp` fields must use simulation time.
6. Test rotor thrust and hover independently before running the MATLAB online control loop.

The FRD rotor coordinates consistent with the mixing matrix are:

| Rotor | FRD position | Yaw reaction-torque sign |
|---|---|---|
| T0 | `(-dx,+dy,0)` | Negative |
| T1 | `(+dx,+dy,0)` | Positive |
| T2 | `(+dx,-dy,0)` | Negative |
| T3 | `(-dx,-dy,0)` | Positive |

Thrust acts along negative body z. `dx` and `dy` are coordinate offsets; 0.225 m must not also be interpreted as the radial distance from the center to a rotor.

The default initial NED heading is approximately 53.13°, corresponding to approximately 36.87° about the world z-axis for an ENU/FLU model. If the model uses different axes, use the matrix transformation rather than these example angles.

## Topics

| Default topic | ROS 2 type | Direction | Data requirements |
|---|---|---|---|
| `/quad/imu` | `sensor_msgs/msg/Imu` | Gazebo → MATLAB | Specific force in m/s² and angular velocity in rad/s, 100 Hz |
| `/quad/position` | `geometry_msgs/msg/PointStamped` | Gazebo sensor/adapter → MATLAB | Local ENU position in meters, 20 Hz, standard deviation 0.02 m |
| `/quad/truth` | `nav_msgs/msg/Odometry` | Gazebo → MATLAB | Ground truth; pose in ENU and twist in the child/body frame |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → MATLAB | Simulation time |
| `/quad/thrust_cmd` | `std_msgs/msg/Float64MultiArray` | MATLAB → actuator adapter | `data=[T0,T1,T2,T3]` in newtons |

MATLAB API type names are commonly written in two-part form, such as `sensor_msgs/Imu`. This represents the same message type as the three-part name used by the ROS CLI.

**The thrust command topic is not a native Gazebo motor-speed topic.** The external adapter must either apply the four thrusts at their corresponding rotor locations or convert them to the motor plugin's required speeds using a calibrated relationship such as `Omega=sqrt(T/k_f)`. Record the array order, `k_f`, `c_tau`, and rotation directions explicitly. If the motor plugin includes dynamic lag, represent it in the prediction model or document and validate it as a model mismatch.

Sensor subscription QoS is best effort/volatile. Command publication is reliable/volatile; the external command subscriber must use compatible QoS. The interface uses standard messages, so custom ROS message support generation is normally unnecessary.

## Coordinate Transformations

The default external world frame is ENU and the external body frame is FLU. Internally, the implementation uses NED and FRD:

```text
W = [0 1 0; 1 0 0; 0 0 -1]
B = diag([1 -1 -1])
p_NED = W * p_ENU
omega_FRD = B * omega_FLU
R_NED_FRD = W * R_ENU_FLU * B'
```

`nav_msgs/Odometry.twist` must follow the message convention and be expressed in the child/body frame. If a Gazebo plugin outputs world-frame velocity, convert it in the adapter first. ROS quaternion fields are read as x/y/z/w and reordered internally to w/x/y/z.

If the GNSS sensor outputs latitude, longitude, and altitude, convert them on the Gazebo/ROS side to local metric coordinates referenced to the simulation world origin, then publish `PointStamped`. Do not feed latitude and longitude into the EKF as meters. Alternatively, simulate GNSS using ground-truth position plus noise, explicitly document that simplification, and keep the uncorrupted truth stream separate from the estimator input.

The IMU `orientation` field is ignored. It must not provide an additional perfect attitude measurement. Initial attitude is supplied by the configured known spawn attitude.

## Starting MATLAB

```matlab
c = project_config;
c.ros.domainID = 0;
c.ros.requireTruth = true;  % Require ground truth for evaluation plots.
session = main_gazebo(c);
```

At startup, MATLAB sends hover thrust and waits for sensor, position, and clock data. Configure the external adapter so it does not continue indefinitely on an old command before startup or after disconnection.

On normal completion or an error exit, the function sends zero thrust to all four rotors. This is a simulator shutdown command, not a valid in-mission flight input. Control stops at the end of the task; an automatic landing is not implemented.

## Synchronization and Timing Limits

Sensor data are ordered by timestamp using a 20 ms simulation-time reorder buffer. Measurements that arrive too late are discarded and counted; old position measurements are not fused repeatedly. The code monitors IMU gaps, missing GNSS data, clock rollback, and message backlog. On a detected limit violation, it saves the interrupted-session log and exits.

This is asynchronous co-simulation with backlog detection, **not lockstep execution**. Reduce Gazebo's real-time factor when NMPC cannot keep up. Strict lockstep operation would require a Gazebo-side coordination service that advances a fixed number of simulation steps after acknowledging each control command, together with corresponding changes to the MATLAB entry point. Adding `pause` alone does not implement lockstep synchronization.

After resetting the model or `/clock`, stop and restart the MATLAB online entry point. Do not change coordinate conventions or sensor time sources during a run.

## Data and Acceptance Checks

`gazebo_session.mat` stores estimates, thrust commands, solver status, ground truth, and packet counters. `analyze_gazebo` interpolates ground truth to controller timestamps, excludes preparation and transition phases, and generates evaluation plots. Quaternion interpolation first enforces sign continuity and then normalizes the interpolated result. This approach is intended for high-rate ground-truth samples, not sparse samples separated by large rotations.

If true sensor biases are unavailable, online bias-error plots contain NaN rather than invented accuracy values. If ground truth is unavailable, the online data can be saved, but true tracking and estimation errors cannot be computed. Online metrics cover only the actual recorded range; check `scoredFrom` and `scoredTo` against the required mission duration.

Validate the external setup in this order:

1. Topics and QoS compatibility.
2. Timestamps and simulation clock.
3. Axis signs and coordinate transformations.
4. Static IMU specific force.
5. Individual rotor torque directions.
6. Hover behavior.
7. Short trajectory tracking.
8. The complete 60-second mission.

Passing the MATLAB self-tests does not imply that these external-system checks have passed.

## Related Documents

- [English project guide](README_EN.md)
- [Implementation notes](DESIGN_NOTES_EN.md)
- [Validation record](VALIDATION_EN.md)
- [Chinese version](GAZEBO_INTERFACE.md)
