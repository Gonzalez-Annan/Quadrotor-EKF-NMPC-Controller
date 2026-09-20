# 实现与推导说明

## 1. 刚体模型

状态 x=[p,v,q,w]，p 和 v 在 NED，w 在 FRD，q 为 FRD 到 NED 的旋转。R(q) 为对应旋转矩阵。

```text
p_dot = v
v_dot = [0,0,g]' - R(q) * [0,0,sum(T)]' / m
q_dot = 0.5 * q (Hamilton product) [0,w]'
w_dot = inv(J) * (tau - cross(w,J*w))
```

混控：[总推力;tau_x;tau_y;tau_z] = mix * [T0;T1;T2;T3]。

roll 力矩行 dy[-1,-1,1,1]，pitch 行 dx[-1,1,1,-1]，yaw 行 c_tau[-1,1,-1,1]。悬停时每路 T=mg/4=2.943 N。

自检分别验证零推力自由落体、悬停平衡和差动推力力矩方向。模型不含阻力、电机滞后和风；这些不是原任务已提供的标称参数，Gazebo 引入时必须额外校准/建模。

## 2. 参考状态

位置、速度、加速度按任务解析公式计算。偏航 psi=atan2(v_E,v_N)。为了避免把滚转俯仰一律设零而与转弯所需加速度冲突：

```text
b3 = normalize([0,0,g]' - a_ref)
heading = [cos(psi), sin(psi), 0]'
b2 = normalize(cross(b3, heading))
b1 = cross(b2, b3)
R_ref = [b1,b2,b3]
```

角速度参考由 R_ref' * Rdot_ref 的反对称部分取得，Rdot 用小时间间隔中心差分。轨迹平动速度与加速度仍使用解析导数。姿态参考仅用于代价和初始化，实际运动由动力学产生。

## 3. EKF

采用右乘姿态误差：q_true=q_hat * Exp(dtheta)。nominal=[p,v,q,ba,bg]，error=[dp,dv,dtheta,dba,dbg]。

```text
a_corrected = a_measurement - ba_hat
w_corrected = w_measurement - bg_hat
a_world = R(q_hat)*a_corrected + gravity
```

传播使用半步姿态转换比力、二阶位置积分、四元数指数更新。线性化连续误差方程的非零块：

```text
F_pv = I
F_vtheta = -R*skew(a_corrected)
F_vba = -R
F_thetatheta = -skew(w_corrected)
F_thetabg = -I
```

状态转移 Phi=I+F*dt+(F*dt)^2/2。采样噪声对速度和姿态的增量分别按 sigma_a*dt、sigma_w*dt；偏置游走按 density*sqrt(dt)。这些不能混作同一个连续噪声参数。

位置更新 H=[I,0]，R_pos=sigma_p² I。K=P H'/(H P H'+R_pos)。使用 Joseph 形式更新 P，然后把局部姿态误差注入四元数，并按 I-skew(dtheta)/2 重置该协方差块。所有 P 更新后对称化，测试检查最小特征值。

估计误差图采用真值减名义值，姿态误差采用 Log(q_hat^-1*q_true)。名义 q 与 -q 等价，不能直接相减定义姿态误差。

±2σ 覆盖率为单条仿真轨迹上的描述性统计，不是经过多次 Monte Carlo 推断的校准保证。偏置与航向可观测性依赖运动激励与初始先验。

## 4. NMPC 数值问题

默认 N=20，Ts=0.05 s，6 个分块时长 [1,1,2,3,5,8]。每个块有 4 个独立变量。优化变量通过可逆线性变换映射为 4 路推力，以改善数值条件：

```text
M = mix \ diag([mass; Jxx; Jyy; Jzz])
T = T_hover + M*z
```

这是输入重参数化，不是用角加速度控制器替代四旋翼动力学。T 仍进入完整非线性模型。物理推力上下限作为 z 的线性不等式进入 fmincon，没有后置限幅来掩盖优化违反。

阶段代价包含位置和速度加权平方误差、符号不变姿态误差 `1-(q'*q_ref)^2`、角速度误差、推力偏离悬停、推力变化。末端状态误差乘 4。时间积分采用 Ts 加权，权重见配置文件。

约束在每个 RK4 子步末端施加。每步四元数归一化，不再引入冗余范数等式约束。姿态约束用旋转矩阵表达：

```text
 R32 - tan(limit)*R33 <= 0
-R32 - tan(limit)*R33 <= 0
 R31 - sin(limit) <= 0
-R31 - sin(limit) <= 0
```

前两个不等式联合保证 R33 非负并限制 roll；结合 pitch 上下界，在正常姿态分支上等价于分别限制 roll/pitch。角速度逐轴标准化到 [-1,1]。

目标与约束的复步梯度由一批虚部扰动轨迹同时计算。预测路径中使用 `sqrt(sum(q.^2))` 而不是带共轭的范数，避免破坏解析延拓；代价也使用非共轭平方和。输出真实目标给 fmincon，虚部仅用于导数。`run_tests` 使用独立中心差分检查梯度。

上一序列向前移位，再按分块起点取样为下一次初值。虽然这减少迭代，分块会限制控制自由度；可增大块数提高精度，但也会增加计算量。

## 5. 分析与边界

推力约束由优化强制，实际状态约束仍需逐样本检查。中间积分节点没有数学上的连续时间绝对保证；输出真值以 100 Hz 记录、内部物理积分 500 Hz。若要严格分析连续时间越界，应额外保存所有物理子步。

计算耗时包括一次优化和其函数/梯度调用，不包括绘图、文件输出，也不代表整个在线通信周期。报告另外列总墙钟时间。ROS 在线运行需留通信裕量，离线完成任务不能证明实时性。

平面轨迹的导数、NED 符号、力矩映射、四元数约定、EKF 重置和物理单位，是小组答辩应能解释的核心内容。
