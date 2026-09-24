function result=run_simulation(c)
%RUN_SIMULATION 执行可复现的 EKF 与约束 NMPC 闭环离线仿真。
% 输入 c：项目配置结构体；省略时使用 project_config 的默认配置。
% 输出 result：包含 log、config、metrics 和 outputDir 的结果结构体。
% 示例：result = run_simulation();
%       c=project_config(); c.duration=3; result=run_simulation(c);
% 一、准备配置和运行环境
% 设置项目路径，确保调用当前项目中的函数。
setup_project();
% nargin 是输入参数数量；未传入 c 时采用默认配置。
if nargin==0, c=project_config(); end
% 将统一配置 c 转成小组成员动力学、传感器和 EKF 所使用的参数结构 p。
p=integration.to_params(c);
% NMPC 使用 fmincon 求解优化问题；检查优化函数是否存在。
assert(exist('fmincon','file')==2,'Optimization Toolbox is required.');
% 反馈模式：ekf 使用估计状态；truth 使用真实状态，作为对照实验。
assert(any(strcmp(c.feedback,{'ekf','truth'})),'feedback must be ekf or truth.');
% 检查周期的整数倍关系，保证控制、GNSS 和动力学积分与主时间轴对齐。
% 默认主步长为 0.01 s，控制及 GNSS 周期为 0.05 s，动力学子步长为 0.002 s。
assert(abs(c.controlDt/c.dt-round(c.controlDt/c.dt))<1e-9);
assert(abs(c.gnssDt/c.dt-round(c.gnssDt/c.dt))<1e-9);
assert(abs(c.dt/c.physicsDt-round(c.dt/c.physicsDt))<1e-9);
% 固定随机种子，使相同配置下的传感器噪声等随机序列可复现。
rng(c.seed,'twister');
% 二、建立本次实验的输出目录
% 未指定实验名称时使用时间戳；固定名称会复用同名目录。
if isempty(c.runName), c.runName=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')); end
outdir=fullfile(c.outputRoot,c.runName); if ~isfolder(outdir), mkdir(outdir); end
% 三、初始化时间轴、参考轨迹、真实状态及估计器
% t 为采样时刻，N 为采样点数；默认 0:0.01:60，共 6001 个点。
% ref 保存各时刻的参考状态，用于初始化和结果评价。
t=0:c.dt:c.duration; N=numel(t); ref=quad.reference(t,c);
% 直接从运动参考轨迹的初始状态开始，本函数不包含地面起飞过程。
% x 为 13 维真实状态：[位置(3); 速度(3); 四元数(4); 机体系角速度(3)]。
% e 为 EKF 内部结构；true 表示按配置加入初始估计误差。
% ctrl 保存 NMPC 内部数据，包括后续优化所用的初始化信息。
x=ref.x(:,1); e=quad.ekf_init(x,c,true); ctrl=quad.mpc_init(c);
% 六维真实传感器偏置：前三维为加速度计偏置，后三维为陀螺仪偏置。
bias=[c.sensor.initialAccelBias;c.sensor.initialGyroBias];
% 四、预分配日志，避免循环中不断扩展数组
% L 为日志结构；状态矩阵每列对应一个时刻。NaN 表示该时刻尚无数据。
% truth、estimate、reference 分别记录真实状态、估计状态和参考状态。
L.t=t; L.truth=nan(13,N); L.estimate=nan(13,N); L.reference=ref.x;
% u：四旋翼推力，单位 N；bias：六维真实偏置。
% error、sigma：15 维估计误差及标准差；姿态误差用三维旋转误差表示。
L.u=nan(4,N); L.bias=nan(6,N); L.error=nan(15,N); L.sigma=nan(15,N);
% 记录 GNSS 测量、归一化创新平方 NIS，以及当前测量是否被拒绝。
% 无 GNSS 更新的时刻保留 NaN；gnssRejected 为逐时刻的逻辑标志。
L.positionMeasurement=nan(3,N); L.nis=nan(1,N); L.gnssRejected=false(1,N);
% 记录优化耗时（秒）、求解退出标志和目标函数值；仅在控制更新时填写。
L.solveTime=nan(1,N); L.exitflag=nan(1,N); L.cost=nan(1,N);
% 记录预测解的约束违反程度、是否启用备用控制输入，以及优化迭代次数。
L.predictedViolation=nan(1,N); L.fallback=false(1,N); L.iterations=nan(1,N);
% 初始输入设为悬停推力；两次控制更新之间保持最近一次推力不变。
u=c.hover; k=1;
fprintf('Run %s: %.2f s, %s feedback, %d prediction steps\n',c.runName,c.duration,c.feedback,c.mpc.horizon);
% 开始测量实际运行耗时；这里的计时不同于无人机的仿真时间。
wall=tic;
% 五、闭环仿真主循环；若循环中出错，catch 会保存现场。
try
    % 本轮开始时，x 和 e 对应 t(k)；本轮末尾推进到下一时刻。
    for k=1:N
        % 5.1 到达 GNSS 采样时刻时，用位置测量修正 EKF。
        % 默认 k=1,6,11,... 时执行，即每 5 个主步长更新一次。
        if mod(k-1,round(c.gnssDt/c.dt))==0
            % 由真实位置生成带噪声的位置测量 z；控制器不会直接使用这份真实状态。
            z=simulateGNSS(x,p);
            % 先保存累计拒绝次数，再执行位置测量更新及异常值门限检查。
            oldRejected=e.gnssRejected; e=quad.ekf_update(e,z,c);
            L.positionMeasurement(:,k)=z; L.nis(k)=e.lastNIS;
            % 累计次数增加，说明当前这次 GNSS 测量被拒绝。
            L.gnssRejected(k)=e.gnssRejected>oldRejected;
        end
        % 5.2 提取 EKF 的 13 维无人机状态估计，供控制器使用。
        xhat=quad.ekf_state(e);
        % 仅在控制周期到来时求解 NMPC；终点不再计算下一段控制。
        % 默认 60 秒任务执行 1200 次控制更新。
        if k<N && mod(k-1,round(c.controlDt/c.dt))==0
            % 通常采用 xhat 闭环控制；只有 truth 对照模式使用真实状态 x。
            if strcmp(c.feedback,'truth'), feedback=x; else, feedback=xhat; end
            % 预测未来运动并求解约束优化；返回当前四旋翼推力、控制器数据和求解信息。
            [u,ctrl,info]=quad.mpc_step(feedback,t(k),ctrl,c);
            % 保存本次优化的性能信息，以便统计收敛情况和实时计算能力。
            L.solveTime(k)=info.solveTime; L.exitflag(k)=info.exitflag;
            L.cost(k)=info.cost; L.predictedViolation(k)=info.violation;
            L.fallback(k)=info.fallback; L.iterations(k)=info.iterations;
        end
        % 5.3 在推进动力学之前，记录当前 t(k) 的同步状态和输入。
        L.truth(:,k)=x; L.estimate(:,k)=xhat; L.u(:,k)=u; L.bias(:,k)=bias;
        % 将估计值与真实状态、真实偏置比较，得到误差状态向量。
        L.error(:,k)=quad.estimation_error(e,x,bias);
        % P 的对角线是各误差分量的方差，开平方得到标准差。
        % max(0,...) 避免极小的数值负值导致复数；绘图用 sigma 构造 ±2σ 界限。
        L.sigma(:,k)=sqrt(max(0,diag(e.P)));
        % 终点只记录数据，不再推进到仿真时长之外。
        if k==N, break; end
        % 5.4 先用 RK4 估算半步后的真实状态，作为本区间 IMU 测量的采样位置。
        % ceil 向上取整确定半步积分的子步数，实际子步长由总时长除以子步数得到。
        mid=integration.plant_step(x,u,c.dt/2,p,ceil(c.dt/(2*c.physicsDt)));
        % 根据中间状态及其动力学变化率，生成加速度计比力和陀螺仪角速度测量。
        % 保留原传感器函数的顺序：先推进随机游走偏置，再构造测量。
        % baNext、bgNext 是更新后的真实偏置；imu 含噪声及偏置。
        [imu,baNext,bgNext]=simulateIMU(mid,quadrotorDynamics(mid,u,p), ...
            bias(1:3),bias(4:6),c.dt,p);
        % 5.5 从本轮起始 x 出发推进完整主步长，而不是从 mid 再推进完整步长。
        % 默认用 5 个 0.002 s 的 RK4 子步推进 0.01 s，并归一化四元数。
        x=integration.plant_step(x,u,c.dt,p,round(c.dt/c.physicsDt));
        % EKF 使用带噪声的 IMU 测量预测下一时刻状态和误差协方差。
        e=quad.ekf_predict(e,imu.accel,imu.gyro,c.dt,c);
        % 同步更新真实传感器偏置，供下一轮记录和生成测量使用。
        bias=[baNext;bgNext];
        % 若真实状态或协方差出现 NaN/Inf，立即中止并保存错误现场。
        assert(all(isfinite(x))&&all(isfinite(e.P(:))),'Simulation produced nonfinite state.');
        % 5.6 大约每 5 秒仿真时间打印进度，并处理 MATLAB 界面刷新。
        % 此处 x 已推进一步，而参考仍为 t(k)，显示误差存在一个主步长的时间错位。
        % 最终性能指标由前面记录的同步日志计算，不受这条进度显示影响。
        if mod(k,round(5/c.dt))==0
            fprintf('  t=%.1f s, position error=%.3f m, elapsed=%.1f s\n',t(k),norm(x(1:3)-ref.p(:,k)),toc(wall));
            drawnow limitrate;
        end
    end
% 六、异常处理：保存已有日志、配置、出错步数及异常对象，再报告错误。
catch ME
    save(fullfile(outdir,'failed_run.mat'),'L','c','k','ME');
    rethrow(ME);
end
% 七、整理输出：完整日志、实际配置和本次结果目录。
result.log=L; result.config=c; result.outputDir=outdir;
% 计算跟踪/估计误差、约束违反次数及求解性能。
% totalWallTime 在导出及绘图之前记录，不包含后续文件导出和视频生成耗时。
result.metrics=quad.metrics(L,c); result.metrics.totalWallTime=toc(wall);
% 保存可在 MATLAB 中重新加载的完整结果结构体。
save(fullfile(outdir,'result.mat'),'result','-v7');
% 导出配置及指标 JSON，以及逐时刻遥测 CSV。
quad.export_results(result);
% 按配置生成五组结果图，保存为 PNG、SVG 和 MATLAB FIG。
if c.makePlots, quad.plot_results(result); end
% 按配置生成 MATLAB 飞行可视化视频；默认关闭。
if c.makeVideo, quad.make_video(result); end
% 在命令窗口展示本次性能指标和保存位置。
disp(result.metrics); fprintf('Saved: %s\n',outdir);
end
