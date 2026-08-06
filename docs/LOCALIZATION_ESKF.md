# RobotCore 固定延迟 ESKF：模型、可观测性与准确性边界

## 结论

当前 C++ 实现是一个数学上自洽的 15 维误差状态卡尔曼滤波器：名义状态在
SO(3) 上传播，协方差按同一线性化点传播；观测通过 NIS 门控，更新采用
Joseph 形式，并在误差注入后执行姿态协方差重置、对称化和正半定投影。
因此，在时间戳、坐标系、噪声统计和观测模型成立时，它是局部无偏且一阶
一致的估计器。

这不是“无条件准确”的证明。绝对精度必须由独立地面真值验证；尚未执行的
真机 RMSE、延迟或漂移项目不能由滤波器结构本身推出。

## 状态与传播

名义状态和右乘误差定义为

\[
x=(p,v,q,b_g,b_a),\qquad
\delta x=(\delta p,\delta v,\delta\theta,\delta b_g,\delta b_a)\in\mathbb R^{15}.
\]

坐标约定为 `odom`/FLU 中的 \(p,v\)，姿态 \(q=R_{ob}\) 将
`base_link` 向量旋转到 `odom`。IMU 给出机体系比力和角速度：

\[
\omega=\omega_m-b_g-n_g,\qquad f=a_m-b_a-n_a,
\]
\[
\dot p=v,\quad \dot v=R(q)f+g,\quad
q_{k+1}=q_k\operatorname{Exp}(\omega\Delta t),
\]

其中 \(g=(0,0,-9.80665)^T\)。实现使用相邻 IMU 的中点值和中点姿态：

\[
p_{k+1}=p_k+v_k\Delta t+\tfrac12a_k\Delta t^2,
\quad v_{k+1}=v_k+a_k\Delta t.
\]

误差状态连续时间雅可比的非零块为

\[
F_{pv}=I,\quad F_{v\theta}=-R[f]_{\times},\quad F_{vb_a}=-R,
\]
\[
F_{\theta\theta}=-[\omega]_{\times},\quad F_{\theta b_g}=-I.
\]

噪声输入矩阵对应陀螺、加速度、陀螺偏置随机游走和加速度偏置随机游走。
离散状态转移使用

\[
\Phi\approx I+F\Delta t+\tfrac12F^2\Delta t^2,
\quad P^- = \Phi P\Phi^T+GQ_cG^T\Delta t.
\]

当启动校准没有得到可信加速度标定时，代码显式令 `accel_valid=false`：只传播
姿态，平移由 VIO 约束，不会把未经验证的加速度当成有效信息。

## VIO 更新和数值稳定性

ZED 适配器先用 TF 把相机/传感器位姿和速度统一成
`odom -> base_link`。位姿残差为

\[
r_p=p_z-p,\qquad r_R=\operatorname{Log}(q^{-1}q_z),
\]

速度先从 base/FLU 旋转到 `odom`，残差为 \(r_v=v_z-v\)。对应观测矩阵
分别选择 \((\delta p,\delta\theta)\) 和 \(\delta v\)。每次更新计算

\[
S=HPH^T+R,\quad \mathrm{NIS}=r^TS^{-1}r,
\]

6 维和 3 维默认门限分别为 22.458 和 16.266。通过门控后：

\[
K=PH^TS^{-1},
\]
\[
P^+=(I-KH)P(I-KH)^T+KRK^T.
\]

误差注入名义状态后，姿态块以
\(I-\tfrac12[\delta\theta]_\times\) 重置。最后归一化四元数、强制协方差
对称，并把特征值下限限制为 \(10^{-12}\)。这些步骤避免普通
\((I-KH)P\) 更新、浮点非对称或四元数漂移造成负方差。

## 固定延迟与 AprilTag

估计器保存 3 秒 IMU 状态/增量，以及一个按时间排序的 VIO/AprilTag 位姿观测队列。
历史项保留传播后的预测状态和该时刻全部视觉更新后的状态。延迟观测到达后选择
时间上最近且误差不超过 120 ms 的历史状态，从该点按时间顺序重做 Tag/VIO 更新，
再重放其后全部 IMU 和视觉观测；窗口外数据丢弃并计数。60 Hz 发布时只在最新
IMU 不超过 50 ms 时前推至当前时间。

AprilTag 初次出现时，在对应历史状态计算

\[
T_{map,odom}=T_{map,base}^{tag}(T_{odom,base}^{eskf})^{-1}.
\]

四帧一致候选只用于建立一次 `map -> odom` 坐标变换。建立后该变换保持固定，
每个通过 PnP 几何门的 `map -> base_link` 观测及其 6x6 协方差被转换到 ESKF 的
`odom` 坐标，和 VIO 一起进入同一固定延迟队列、NIS 门控和 15 维状态更新。
因此不存在隐藏在 `map -> odom` 中的第二套平滑器；Tag 是绝对位姿约束，VIO 是
局部位姿/速度约束，IMU 负责传播。

ESKF 只订阅 `/localization/apriltag_pose` 一条 AprilTag 输入。其
`AprilTagPoseEstimate` 同时携带位姿、协方差、质量/拒绝原因和
`map_generation`；地图重载时定位器在同一消息中递增代次并请求重新对齐，
不再使用独立的 `pose_status` 或 `relocalize_event` 话题。

## 数据来源与实际作用

| 来源 | 频率目标 | 进入估计器的量 | 失效行为 |
|---|---:|---|---|
| A-board UART8 外置 IMU | 100 Hz | 角速度、标定后比力、MCU 时间 | 50 ms 后转 VIO-only；无可信六面标定时禁用加速度 |
| ZED VIO | 30 Hz | `odom` 位姿和 base 速度及协方差 | 最多 0.5 s 惯性外推，之后位置无效 |
| Isaac ROS AprilTag CUDA | 30 Hz | 联合 PnP 绝对位姿及 6x6 协方差 | 与 VIO 一起在历史时刻直接更新 ESKF；失效时保持局部惯性/VIO 估计 |
| CameraInfo | 启动/低频 | 投影内参 | 无有效内参不发布 Tag 位姿 |
| Tag map | 启动/重定位 | 每个 Tag 的尺寸、地图位姿 | 文件无效时保持旧地图或拒绝定位 |
| 静态 TF/安装标定 | 启动 | 传感器到 `base_link` 外参 | 错误外参产生系统偏差，滤波无法自行消除 |
唯一位姿/速度输出是 `/localization/fused_odom` 和由它同次生成的
`/robot/body_state`。建立绝对对齐后位姿为 map/FLU；此前为 odom/FLU；线速度和
角速度始终为 base_link/FLU。Tag 新鲜且通过质量门时
`state_valid=true`；Tag 遮挡但 VIO/ESKF 仍可用时
`position_estimated=true`。水池底为 `map` 原点时，垂向位置统一使用
`pose.position.z`（FLU，向上为正），不再维护方向相反且未标定的深度字段，
也不保留与池底原点重复的高度占位字段。

## 可观测性

- 连续 Tag 地图观测使全局三维位置与姿态可观；无 Tag 时，`map -> odom` 只能
  保持最近值，无法检测地图坐标系中的整体漂移。
- ZED VIO 提供局部尺度、姿态、位置和速度约束；纹理退化、重复纹理、曝光和
  水下折射会降低这些约束的真实性。
- 静止重力约束能观测横滚、俯仰及陀螺零偏的一部分。仅靠重力不能观测偏航；
  任意运动下加速度偏置与真实线加速度也不能瞬时分离。
- 六面标定固定尺度、非正交和静态偏置。10 秒启动阶段只估计静止零偏和残余
  加速度偏置，不把尺度/安装角等弱可观参数在线“估计成真值”。

## 主要误差来源和可修改点

| 误差来源 | 表现 | 优先修改点 |
|---|---|---|
| 相机、IMU 外参误差 | 转弯时位置出现方向相关偏差 | 标定 `base_to_camera` 和 `base_to_imu`，做 FLU 单轴检查 |
| Tag 地图坐标/尺寸误差 | 绝对位置形成固定或分区偏差 | 用测量基准重建地图，记录每 Tag 尺寸/不确定度 |
| 水下折射、镜罩与 CameraInfo 不匹配 | PnP 深度和边缘位置系统偏差 | 在实际介质/镜罩下重标内参；必要时使用折射模型 |
| UART 时钟/排队延迟 | 高频运动时相位滞后 | 查看 `imu_transport_p95_ms`、序号缺口和队列高水位 |
| IMU 温漂、振动、饱和 | 偏置漂移、速度/位置二次增长 | 温度分段标定、减振、按 Allan 方差调整 Q |
| VIO 协方差过小或状态相关性 | NIS 偏大、门控频繁或估计过度自信 | 用回放 NIS/NEES 标定 R/Q；后续可联合处理 pose/velocity 互相关 |
| 延迟历史按最近 IMU 点匹配 | 最多约一个 IMU 周期的时间量化误差 | 将测量时刻插入 IMU 区间并做分段传播 |
| PnP 像素离群/误识别 | 突发错误全局修正 | 加强字典、边长、独立 Tag、重投影与四帧一致门控 |
| 未建模动力学 | 短时预测误差 | ESKF 不依赖运动模型；需要时增加水动力约束但必须重新做一致性验证 |

当前实现仍把 VIO pose 和 velocity 分两次更新，未使用两者之间的完整交叉协方差；
历史观测也按最近 100 Hz IMU 状态对齐。这两点是下一轮数学一致性改进的最高优先级。
此外，frame 4 没有传感器硬件序列号字段。部署采用一台设备对应
`/etc/robotcore/external_imu_calibration.yaml` 一个文件的约定；节点在每次服务启动时
重新读取它，只检查矩阵、偏置的维度、有限性和非奇异性，不维护无法由协议验证的
序列号或配置哈希副本。文件与物理 IMU 的对应关系由设备部署目录保证。

## 如何判断“准确”

代码级正确性由 golden frame、SO(3)、静止传播、NIS 拒绝、Joseph/PSD 和回放测试
约束；统计一致性应通过 Monte Carlo 的 NIS/NEES；真实准确性只能通过独立地面
真值计算 RMSE、p95、静止波动和遮挡漂移。实际状态见
`docs/LOCALIZATION_CPP_ACCEPTANCE.md`。
