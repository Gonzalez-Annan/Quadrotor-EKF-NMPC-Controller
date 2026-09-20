# MA6224：MATLAB 四旋翼 EKF + NMPC 项目

## 快速开始

在 MATLAB 中打开本文件所在文件夹作为“当前文件夹”，依次执行：

```matlab
check_environment
run_tests

% 先运行短仿真，确认本机速度和绘图正常。
c = project_config;
c.duration = 3;
quick = main_simulation(c);

% 正式作业：规定的 60 秒轨迹，估计状态反馈。
c = project_config;
result = main_simulation(c);

% 用已完成的日志生成视频，不必重新跑优化器。
quad.make_video(result);
```

脚本不需要连接 Gazebo 就能独立运行。Gazebo 接入请先读 `GAZEBO_INTERFACE.md`，不要直接在未知 ROS 网络上运行 `main_gazebo`。

需要 MATLAB 和 Optimization Toolbox。Gazebo 接口还需要 ROS Toolbox。此实现直接使用 `fmincon` 的 SQP 求解带硬约束的非线性预测控制问题，不调用 `nlmpc`，因此不依赖 MPC Toolbox 的具体版本，也没有借用现成飞控闭环代替 NMPC。

## 文件与职责

| 文件 | 职责 |
|---|---|
| `project_config.m` | 全部物理参数、噪声、控制权重、采样周期、ROS 配置 |
| `main_simulation.m` | 独立 MATLAB 闭环仿真入口 |
| `run_tests.m` | 物理平衡、四元数、参考导数、优化梯度、EKF、自稳态控制自检 |
| `run_benchmarks.m` | 真实状态反馈基线与多个噪声种子实验 |
| `main_gazebo.m` | ROS 2 在线收发、状态估计与控制入口 |
| `analyze_gazebo.m` | 对齐 Gazebo 真值与 MATLAB 日志并生成图表 |
| `+quad/dynamics.m`, `step.m` | 13 维刚体动力学与 RK4 |
| `+quad/reference.m` | 规定的三维 8 字轨迹与速度、加速度、姿态参考 |
| `+quad/ekf_*.m` | 乘法误差状态 EKF |
| `+quad/mpc_*.m` | 直接打靶 NMPC、移动分块、梯度、优化和故障记录 |
| `+quad/Math.m`, `frames.m` | 无额外工具箱依赖的旋转与坐标变换 |
| `+quad/plot_results.m`, `make_video.m` | 必需图表与 MP4 |
| `+quad/RosIO.m`, `ros_reference.m` | ROS 2 传输、缓存、启动过渡参考 |

## 输出内容

每次运行写入 `results/<运行标识>/`，默认标识带时间戳。手工指定同名 `c.runName` 会覆盖同目录同名结果，请只在有意替换该实验时使用。

- `result.mat`：完整轨迹、估计、15 维误差协方差对角、偏置真值、控制、求解状态、配置。
- `configuration.json`：全部运行参数。
- `metrics.json`：位置 RMSE / 最大误差、约束违反样本数、耗时、失败计数等。
- `telemetry.csv`：便于汇总的位置、速度、推力、求解日志。
- `01_trajectory.*`：三维参考/真实/估计轨迹。
- `02_tracking_errors.*`：位置、速度、旋转误差、角速度误差。
- `03_ekf_bounds.*`：15 维误差状态及 ±2σ。
- `04_actuators_constraints.*`：推力、力矩、滚转/俯仰和角速度约束。
- `05_solver_performance.*`：真实墙钟求解时间及状态。
- 图表格式为 PNG、SVG 和可编辑 FIG。
- 可选 `flight_visualization.mp4`：MATLAB 生成的飞行与实时误差动画，不冒充 Gazebo 录屏。

中途异常会保存 `failed_run.mat`，保留已完成的数据。未写入的数据为 NaN，不能当作完整实验结果。

## 固定约定

- 世界：NED（北、东、下）。机体：FRD（前、右、下）。
- 状态顺序：`[p(3); v(3); q(4); omega(3)]`。
- 四元数：Hamilton，标量在前 `[w;x;y;z]`，机体到世界的主动旋转。
- 输入：`[T0;T1;T2;T3]`，单位 N，旋翼编号对应课程文档的混控矩阵。
- 推力沿机体系负 z 轴；重力在 NED 中为 `[0;0;9.81]`。
- 单旋翼 0.2～5.5 N，roll/pitch 分别限制 ±35°，角速度限制 `[180;180;90]`°/s。
- 数值计算中所有角度均为 rad；图表按标注转换。

课程文档的加速度公式缺少显式质量因子、推力方向和状态顺序也不够一致。这里按上述物理一致约定实现，通过悬停平衡与混控方向测试；应在最终报告中说明，而非照抄疑似排版错误。

## 传感器与 EKF

IMU 100 Hz，位置 20 Hz；标准差分别为 0.08 m/s²、0.015 rad/s、0.02 m。噪声只添加一次。

EKF 的名义状态为 `[p;v;q;ba;bg]`（16 个存储分量），误差协方差为 `[dp;dv;dtheta;dba;dbg]`（15 维）。四元数的单位范数意味着姿态只有 3 个独立误差自由度。控制器接收的角速度由陀螺仪减去估计偏置得到，组合成所需 13 维状态。

IMU 用于预测，GNSS 用于位置更新；使用 Joseph 协方差更新、姿态误差注入和协方差重置。±2σ 姿态图对应局部旋转向量，不是直接将 3 维角度方差贴到 4 维四元数上。该协方差不包含独立角速度状态，因此不伪造角速度状态的 ±2σ 曲线。

偏置随机游走强度和初始偏置是作业没有指定的建模假设，全部写在配置。给定 IMU 标准差按“每次采样标准差”解释；偏置随机游走参数按每根号秒解释，离散化不同。

位置更新包含异常创新门限，并记录拒绝次数。IMU+位置并非在所有运动下都能充分观测偏航与所有偏置；本实验采用已知初始姿态附近的先验，不声称实现任意未知航向的全局初始化。

## NMPC 实现

- 20 Hz 控制；默认 20 步、1 s 预测。
- 6 个移动分块 `[1 1 2 3 5 8]`，共 24 个优化变量。
- 每个预测周期分两个 RK4 子步；每个子步检查姿态和角速度硬约束。
- 优化变量采用可逆的加速度尺度变换；推力边界作为线性不等式精确进入优化，总推力边界随单旋翼边界自动满足。
- 位置、速度、姿态、角速度、输入偏离悬停和输入变化进入代价；末端放大状态误差。
- 四元数代价与符号无关。姿态参考由参考加速度和偏航构造。
- 使用向量化复步法提供目标/约束梯度，经过中心差分独立验证。
- 上一时刻预测序列移位作为初值。

`exitflag=0` 表示迭代上限，不代表已收敛。只有数值有限且硬约束满足容差的结果才允许使用；日志保留这一退出状态。不可用时短暂保持上一输入，连续失败则停止并保存。保持输入不是安全保证，也不会被隐藏成“求解成功”。

名义 NMPC 约束不保证含噪声、模型误差下的实际状态永不越界；因此指标检查的是仿真真值。没有额外的力矩硬界，课程文件只给出了由旋翼推力产生的力矩关系，图中报告实际力矩。

## 初始条件与实验公平性

独立 MATLAB 标准实验从轨迹 t=0 的位置、速度、姿态、角速度初始化；估计器另加明确的小初始误差。轨迹 t=0 有非零速度，因此这不是“从地面起飞”。用已知初始条件初始化之后，EKF 不读取真值。

Gazebo 默认从空中静止悬停开始，先预备，再用五次多项式过渡到规定轨迹的初始位置、速度和加速度。正式评分排除预备与过渡段，另跑 60 s。Gazebo 模型必须事先生成在配置的初始位置和航向。

为比较控制器与估计器，可执行：

```matlab
results = run_benchmarks([6224 6225 6226]);
```

真实状态反馈仅为基线，正式结果使用 `c.feedback='ekf'`。不同噪声种子检验重复性，不是课程已规定的额外评分项。

## 耗时与 Gazebo 节拍

求解时间用 `tic/toc` 测量墙钟耗时，50 ms 为控制周期。若耗时大于 50 ms，离线仿真仍可运行，但不能声称实时。不要将“60 s 仿真时间”混同“60 s 墙钟时间”。

Gazebo 必须降低实时因子，使 MATLAB 处理得过来；建议先设置很低的实时因子，再根据最坏求解时间和通信耗时调整。ROS 桥接不自动提供锁步，当前在线入口监测积压、传感器缺失和时钟回退，超限会停止而不是继续使用过时状态。

## 学习和报告

请阅读 `DESIGN_NOTES.md` 理解算法公式，并阅读 `VALIDATION.md` 区分实际完成的测试与未完成的外部集成。报告仍需你们根据课程评分细则撰写、解释参数选择与局限，不能把未经运行的配置描述成实验结果。

官方参考：

- [fmincon 与梯度接口](https://www.mathworks.com/help/optim/ug/fmincon.html)
- [ROS 2 subscriber 与单参数回调](https://www.mathworks.com/help/ros/ref/ros2subscriber.html)
- [ROS Toolbox 支持的发行版](https://www.mathworks.com/help/ros/gs/ros-system-requirements.html)
- [Gazebo / ROS 2 版本搭配](https://gazebosim.org/docs/harmonic/ros_installation/)
