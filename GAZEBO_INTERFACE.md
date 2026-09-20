# Gazebo 与 MATLAB 的接口契约

MATLAB 侧代码已提供；Gazebo 世界、SDF、传感器、ROS 桥接和旋翼执行器适配器需要在 Ubuntu / Gazebo 端准备。本项目不把尚未连接的 Gazebo 当作已经验证。

## 版本与运行前条件

建议 MATLAB R2026a + ROS 2 Jazzy + Gazebo Harmonic + Ubuntu 24.04。MATLAB 可以在 Windows；网络需支持双向 DDS 发现与数据通信。两端 ROS_DOMAIN_ID 一致，ROS_LOCALHOST_ONLY 不能阻止跨机通信。按照本机网络状况配置 DDS；不在脚本中修改防火墙。

1. 建立质量 1.20 kg、惯量 `[0.0125 0.0125 0.0220]` kg·m² 的四旋翼。
2. 在初始 ENU 位置 `[0,0,2]` m 生成无人机，设置与 `c.ros.initialQuaternion` 对应的姿态。
3. 对照 `c.mix` 校准编号、转向和反扭矩，不要直接套现成无人机的旋翼编号。
4. 安装 IMU 和 20 Hz 本地位置测量；按作业添加一次噪声和偏置。
5. 发布 `/clock`，所有测量 header.stamp 必须是仿真时间。
6. 先单独测试四旋翼推力与悬停，再运行 MATLAB 在线闭环。

与混控矩阵一致的 FRD 旋翼坐标为：T0=(-dx,+dy,0)，T1=(+dx,+dy,0)，T2=(+dx,-dy,0)，T3=(-dx,-dy,0)，推力沿 -z；偏航反扭矩符号依次为 [-,+,-,+]。dx、dy 是坐标位移，不能把 0.225 m 同时当作从中心到旋翼的径向长度。

默认 NED 初始航向约 53.13°，对应 ENU/FLU 模型绕世界 z 轴约 36.87°。若使用不同模型轴向，应以矩阵转换为准。

## 话题

| 默认话题 | ROS 2 类型 | 方向 | 数据要求 |
|---|---|---|---|
| `/quad/imu` | `sensor_msgs/msg/Imu` | Gazebo → MATLAB | 比力 m/s²，角速度 rad/s，100 Hz |
| `/quad/position` | `geometry_msgs/msg/PointStamped` | Gazebo传感器/适配器 → MATLAB | ENU 本地位置 m，20 Hz，标准差 0.02 m |
| `/quad/truth` | `nav_msgs/msg/Odometry` | Gazebo → MATLAB | 真值，pose 在 ENU；twist 在 child 机体系 |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → MATLAB | 仿真时间 |
| `/quad/thrust_cmd` | `std_msgs/msg/Float64MultiArray` | MATLAB → 执行器适配器 | `data=[T0,T1,T2,T3]` N |

MATLAB API 中类型名通常写成 `sensor_msgs/Imu` 等，两段式类型名与 ROS CLI 的三段式名称表示同一消息。

**推力话题不是 Gazebo 自带电机速度话题。** 外部适配器必须把 4 个推力施加到对应位置，或者按已校准的 `Omega=sqrt(T/k_f)` 换成电机插件需要的速度。4 维数组顺序、k_f、c_tau 和旋转方向都需显式记录。若电机插件带动态滞后，应在预测模型中体现，或作为模型误差说明并验证。

订阅 QoS 为 best effort / volatile；命令发布 reliable / volatile。外部命令订阅者使用 compatible QoS。数值不含自定义消息，通常无需生成自定义 ROS 消息支持包。

## 坐标变换

默认外部世界 ENU，外部机体 FLU。内部世界 NED、机体 FRD：

```text
W = [0 1 0; 1 0 0; 0 0 -1]
B = diag([1 -1 -1])
p_NED = W * p_ENU
omega_FRD = B * omega_FLU
R_NED_FRD = W * R_ENU_FLU * B'
```

`nav_msgs/Odometry.twist` 必须依照消息语义使用 child/body frame；若 Gazebo 原始插件输出世界系速度，需要先在适配器转换。四元数输入从 ROS 的字段 x/y/z/w 读取后内部重排为 w/x/y/z。

GNSS 如果原始输出经纬高，需要在 Gazebo/ROS 侧转换到与世界原点一致的本地米制坐标，再发布 PointStamped；不要把经纬度当作米送入 EKF。也可以使用真值位置加噪声来模拟 GNSS，须在报告说明该简化，且不要将未加噪真值流接到估计器。

IMU orientation 字段被忽略。不能让其偷偷提供额外的完美姿态。初始姿态由配置的已知生成姿态给定。

## MATLAB 启动

```matlab
c = project_config;
c.ros.domainID = 0;
c.ros.requireTruth = true;  % 为了得到完整作业评估图表
session = main_gazebo(c);
```

启动后先发送悬停推力，等待传感器、位置和时钟。请提前配置外部适配器：启动前/断连后不依赖旧命令一直飞行。函数正常结束或异常退出会发送四路零推力，这是仿真退出的停机命令，不是任务内的合法飞行输入；任务末尾会停止控制而非自动降落。

## 同步和耗时限制

代码按时间戳整理传感器，使用 20 ms 仿真时间重排缓存。过迟测量会丢弃并计数，不重复融合旧位置。监测 IMU 间隙、GNSS 缺失、时钟回退、消息积压；触发后保存中断日志并退出。

这是带积压检测的异步联合仿真，**不是锁步**。NMPC 较慢时需降低 Gazebo 实时因子。若需要严格锁步，下一阶段应在 Gazebo 端实现“控制命令确认后推进固定步数”的协调服务，并相应修改此入口。不要仅靠 `pause` 假装实现锁步。

模型或 `/clock` 重置后应结束并重新启动 MATLAB 在线入口。仿真过程中不要切换坐标约定或更改传感器时间源。

## 数据与验收

`gazebo_session.mat` 保留估计、推力、求解状态、真值和包计数。`analyze_gazebo` 对真值插值到控制时间戳，排除启动与过渡，再生成图表。姿态插值先处理四元数符号连续性，再归一化；适用于高频真值采样，不用于跨越大角度的稀疏插值。

没有偏置真值时，在线偏置误差图为 NaN，不能虚构偏置精度。没有真值时，只保存在线数据，不能计算真实跟踪或估计误差。在线指标只覆盖实际记录范围，必须检查 scoredFrom/scoredTo 是否覆盖所需任务。

接入前依次验证：话题与 QoS、时间戳、各轴符号、静态 IMU 比力、单旋翼力矩、悬停、短时轨迹、完整 60 s。MATLAB 内通过的自检不等于这些外部系统测试已经通过。
