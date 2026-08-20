# RobotCore ZED VIO + AprilTag 定位

生产定位使用两个职责严格分离的坐标层：

- ZED GEN_3 VIO 是唯一连续局部运动源，输出 `odom -> base_link` 位姿、速度与协方差。
- AprilTag 地图定位只观测 `map -> base_link`，融合节点由同一时刻的 VIO 计算并平滑更新 `map -> odom`。

最终位姿为：

`map -> base_link = (map -> odom) * (odom -> base_link)`

AprilTag 不修改、不重置也不门控局部 VIO。系统不再对外部 IMU 做位置积分，不再包含
15 维 ESKF、固定延迟回放、VIO NIS 拒绝、连续拒绝重锚或多帧等待状态机。ZED SDK
仍在相机内部使用其出厂校准 IMU 完成 VIO；独立 Aquaboard IMU 只供遥测使用。

## 输入与输出

| 话题 | 频率 | 用途 |
|---|---:|---|
| `/zedx/zed_node/odom` | 目标 30 Hz | 唯一局部位姿和速度输入 |
| `/localization/apriltag_pose` | 随检测 | 地图绝对观测 |
| `/localization/fused_odom` | 60 Hz | 对齐并短时外推后的里程计 |
| `/robot/body_state` | 60 Hz | 控制和 UI 的唯一状态输入 |
| `/localization/status` | 10 Hz | 频率、延迟、创新量和健康状态 |

融合节点直接把相机坐标转换到 `base_link`，不再发布中间
`/localization/zed_odom` 话题。

## 时间与有效性

VIO 流新鲜度以回调到达时间判断，测量时间戳只用于对齐和限定外推时长：

- 到达间隔不超过 0.30 s；
- 测量时间年龄不超过 0.50 s；
- 发布周期固定为 60 Hz；
- 每个新的 VIO 测量只接收一次，过期或重复时间戳直接丢弃。

Tag 消失时 `map -> odom` 保持不变，VIO 继续产生连续轨迹。此时
`position_estimated=true`、`state_valid=false`，所以 UI 显示局部估计而不是 `waiting`，
控制逻辑仍能区分它与新鲜绝对定位。VIO 本身过期才表示没有可用位置估计。

## AprilTag 创新门控

Tag 与时间对齐的 VIO 共同生成候选 `map -> odom`。候选位置相对当前对齐超过
0.5 m 时，当前帧直接拒绝；不累计 4 帧、15 帧或任何等待状态。通过门控后，使用
Tag 与 VIO 协方差对 6 自由度对齐量做 Kalman 更新，因此绝对修正平滑作用于
`map -> odom`，局部 VIO 轨迹不会被折弯。

地图在服务启动前必须存在且可读；systemd 的 `ExecStartPre` 负责这一条件。运行中的
地图重载只清空 `map -> odom` 对齐，下一条可信 Tag 立即重新建立对齐，VIO 连续性不受影响。

## ZED 性能配置

相机原生采集 SVGA 960x600、30 Hz，发布缩放为 1.0，深度模式为 NEURAL_LIGHT 以满足
GEN_3 VIO。ROS 不发布 ZED IMU、深度图、点云等无人订阅的数据；SDK 内部 IMU 融合保持启用。
浏览器压缩图像按需订阅，并在 UI 侧限制为 15 Hz。
