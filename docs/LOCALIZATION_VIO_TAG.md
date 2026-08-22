# RobotCore ZED VIO + AprilTag 定位

生产状态估计由单个 `/ekf` 节点完成：

- 状态为位置 `p`、世界系线速度 `v`、姿态 `q` 和机体系角速度 `ω`，误差协方差为 12x12；
- 测量间采用常速度、常角速度模型传播，线加速度和角加速度只作为过程噪声；
- ZED GEN_3 VIO 的位置只更新位置状态，机体系 twist 单独更新速度状态；
- AprilTag `map -> base_link` 先建立固定 `map -> odom`，随后作绝对位姿更新；
- 延迟 VIO 和 Tag 按 Header 时间戳插入三秒测量历史，从历史锚点重放；
- Aquaboard 外部 IMU 不订阅进该节点，也不进入 EKF 状态或协方差。

最终位姿为：

`map -> base_link = (map -> odom) * (odom -> base_link)`

`BodyState.pose.orientation` 是定位坐标变换的一部分，来自 ZED/Tag EKF；控制器只用它把
地图位置误差和地图线速度转换到 `base_link`，不把它作为姿态反馈。姿态误差和角速度反馈
由 PID 直接订阅 `/sensors/external_imu`。轨迹节点也直接从该话题锁存定点模式航向，因此
外置 IMU 与定位 EKF 完全解耦。

VG/AH/MINS 数字协议的 `0x06` 48 字节浮点帧已经给出 PITCH、ROLL、YAW，所以 Aquaboard
直接转发设备原生姿态，不运行 Madgwick、Mahony 或其他软件 AHRS。ZED SDK 内部继续用
相机自带 IMU 生成完整 VIO；这与独立的控制姿态反馈不是同一条链路。

## 输入与输出

| 话题 | 频率 | 用途 |
|---|---:|---|
| `/zedx/zed_node/odom` | 目标 30 Hz | 局部位姿、机体系 twist 及协方差 |
| `/zedx/zed_node/pose/status` | 目标 30 Hz | 只接收 `odometry_status=OK` 的 VIO |
| `/localization/apriltag_pose` | 随检测 | 地图绝对位姿及协方差 |
| `/sensors/external_imu` | 目标 100 Hz | 绕过 EKF，直接供 PID、轨迹航向和 UI 使用 |
| `/robot/body_state` | 60 Hz | EKF 定位状态输出；姿态仅用于定位坐标变换 |
| `/localization/status` | 60 Hz | 后验协方差、频率、延迟、创新量和健康状态 |

融合节点直接把相机坐标转换到 `base_link`，不发布中间
`/localization/zed_odom` 话题。

## 协方差

ZED SDK 5.4 的 `sl::Pose` 提供 `pose_covariance[36]` 和
`twist_covariance[36]`。本机 ROS2 wrapper 将速度协方差按 camera-to-base 六维 twist 雅可比
`J Σ Jᵀ` 写入 `/odom.twist.covariance`。

融合层直接使用 ZED 报告的协方差，但对线速度标准差设置 `0.10 m/s` 下限，不保留缺失
协方差时的经验回退：

- 位姿协方差非有限或非正定时，整条 VIO 测量无效；
- twist 协方差非有限或非正定时，只跳过该次 twist 更新，位姿仍可更新；
- Tag 协方差非有限或非正定时，拒绝该次 Tag 更新；
- VIO 位置、姿态、线速度、角速度以及 Tag 位置、姿态均按三维分量做 NIS 门控，默认
  99.9% 卡方阈值为 16.266；
- VIO 与 Tag 的位置测量只修正位置，不利用位置—速度交叉协方差间接修正速度；同一条 ZED
  odometry 中的 twist 是唯一线速度观测，避免把相关的 pose 与 twist 当作独立速度信息；
- VIO 线速度原始创新超过 `0.25 m/s` 时拒绝；通过门控的测量每帧最多修正速度状态
  `0.04 m/s`，触发时同步缩放 Kalman 增益和 Joseph 协方差更新；
- 外置 IMU 协方差不参与定位 EKF。

因此输出速度来自 EKF 状态，不是把相邻帧的原始 ZED 速度直接透传给控制器。过程噪声决定
速度随时间可以变化的快慢，测量协方差决定每次 VIO twist 对速度状态的修正强度。原始
ZED odometry 也写入实验 rosbag，便于把后续速度创新与源测量逐帧对齐。

## 时间与有效性

测量时间戳必须位于当前 ROS 时钟 epoch，并落在三秒测量历史窗口内。VIO 可用性要求：

- 到达间隔不超过 0.80 s；
- 测量时间年龄不超过 0.80 s；
- 新近 ZED tracking status 为 `OK`；
- 每个新的 VIO 时间戳只接收一次，过期或重复时间戳直接丢弃。

Tag 消失时 `map -> odom` 保持不变，VIO 继续产生连续轨迹。此时
`position_estimated=true`、`state_valid=false`，控制逻辑能区分局部估计和新鲜绝对定位。
VIO 本身过期才表示没有可用位置估计。

## AprilTag 融合

启动时连续四个一致候选确定一次 `map -> odom` 及其协方差。用于建立坐标关系的第四帧不再
重复作为 EKF 更新，避免同一观测被计入两次。坐标关系建立后保持固定；后续 Tag 位姿
更新 EKF 的位置和定位姿态，并继续用于计算平移/角度残差和拒绝错误检测。

地图重载会清空受旧地图约束的测量历史，由最近有效 VIO 重新初始化，再用四个一致 Tag
建立新坐标关系，避免旧地图修正残留在状态中。对齐完成后的 Tag 位姿可更新 EKF 的位置
和定位姿态；该定位姿态不进入 PID 的姿态误差或角速度反馈。

## ZED 性能配置

相机原生采集 SVGA 960x600、30 Hz，发布缩放为 1.0，深度模式为 NEURAL_LIGHT，以满足
GEN_3 VIO。ROS 不发布无人订阅的 ZED IMU、深度图和点云；SDK 内部 IMU 融合保持启用。
浏览器压缩图像按需订阅，并在 UI 侧限制为 15 Hz。
