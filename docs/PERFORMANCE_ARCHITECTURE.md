# RobotCore 架构与性能审查

更新时间：2026-08-06。本页区分静态/本机验证与仍需真机满载验证的结论。

## 当前结论

生产定位链已经采用合适的 CPU/GPU 边界，不应整体重写：

```text
ZED 960x600@30
  -> ZED BGR8 NITROS GPU 直连
  -> Isaac ROS CUDA AprilTag
  -> C++ 多 Tag 地图 PnP
  -> C++ EKF Tag 位置延迟测量更新

ZED VIO @30 Hz -> C++ EKF 位姿/twist 观测更新
Aquaboard IMU @100 Hz -> C++ bridge -> PID/轨迹航向直接输入（绕过 EKF）

BodyState @60 Hz
  -> Python 六自由度 PID/推进器分配 @60 Hz（显式启用 pool tracking 时）
  -> Python 单写者命令仲裁 @100 Hz
  -> C++ Aquaboard bridge @50 Hz
  -> MCU PWM/watchdog
```

ZED 直接取 GPU BGR8，cuAprilTags 原生接受该格式，因此图像链不再包含额外格式转换；
AprilTag 检测留在 GPU。6 自由度地图对齐、6x8 推进器分配及安全状态机留在 CPU。
对这些小矩阵使用 GPU 会增加传输、同步和 kernel launch 延迟。

## 理论吞吐与延迟下限

下表是由配置和协议直接得到的物理/调度边界，不是尚未测得的真机承诺。

| 环节 | 固定速率/大小 | 理论边界 |
|---|---:|---:|
| ZED 图像 | 960x600x3, 30 Hz | 帧周期 33.33 ms；未压缩像素流约 51.84 MB/s，必须保持 NITROS/GPU 路径 |
| 外部 IMU | 33 B, 100 Hz | 10 ms 采样周期；115200 8N1 串行时间 2.86 ms |
| IMU+状态同批上行 | 33 B + 48 B | 最坏连续串行时间 7.03 ms |
| 上行 UART 占用 | IMU 100 Hz + 状态 20 Hz + 运行状态 2 Hz | 4334 B/s，占单向 11520 B/s 的 37.6% |
| 下行 UART 占用 | 34 B 命令，50 Hz | 1700 B/s，占 14.8%；单帧串行时间 2.95 ms |
| VIO/Tag fusion、BodyState | 60 Hz | 输出周期 16.67 ms |
| PID | 60 Hz | 调度相位最多 16.67 ms |
| 命令仲裁 | 100 Hz | 调度相位最多 10 ms |
| Aquaboard 发送 | 50 Hz | 调度相位最多 20 ms |

因此最高“有用”执行器命令频率是 50 Hz。提高 Python PID 或仲裁频率不能越过 Aquaboard
50 Hz 边界；仲裁保持 100 Hz 的价值是每个硬件发送周期至少检查两次新鲜度和安全状态。

从一条已经发布的 `BodyState` 到命令完成串行发送，理想相位约为计算时间加 2.95 ms；
独立定时器的保守相位预算约为 PID 16.67 + 仲裁 10 + bridge 20 + 串行 2.95 =
49.62 ms。若后续真机 trace 证明相位等待主导 p99，可让 PID 由新 `BodyState` 事件触发，
或把 PID/仲裁合并为一个 C++ component；在有数据前不应增加这项迁移风险。

## 已实施的最小改动

### 删除影子实现

删除了未被 `robotcore_sensors/CMakeLists.txt` 安装、且已由生产 C++ component 替代的
Python AprilTag、Tag/VIO 对齐、IMU 条件化、ZED 适配、融合和坐标数学实现，以及重复的
Python ZED 子进程启动器。连同只验证旧实现的测试，共减少约 4,500 行。

`robotcore_sensors` 的生产可执行入口现在只有：

- `apriltag_localization_node`
- `vio_tag_fusion_node`

注意：colcon 增量安装不会自动删除已经取消的 console script。部署升级必须使用干净的
包安装空间或显式核对 `ros2 pkg executables robotcore_runtime`，不能继续使用仓库根目录下
第二套陈旧 `build/install/log`。

### 推进器分配

正常未饱和路径预计算阻尼伪逆，每周期只做矩阵乘法；触碰物理推力边界时才调用 SciPy
`lsq_linear(method="bvls")`，获得全局有界最小二乘解，替代只会单向夹紧的自写循环。

本机 5,000 个随机输入微基准：

| 工况 | 修改前 p50 | 修改后 p50 | 说明 |
|---|---:|---:|---|
| 正常范围 | 559 us | 463 us | 约快 17%，这是调参后应长期处于的路径 |
| 大范围、约 95% 饱和 | 855 us | 1086 us | BVLS 较慢，但残差全局最优且不会被错误夹紧状态困住 |

60 Hz 正常控制只占约 28 ms CPU/s。该量级不支持现在把 PID 整体重写为 C++。

### 空闲自动控制图

现场逐进程采样显示，在 PID/池边界配置仍为 fail-closed 时，四个无法产生控制输出的节点
仍合计占约 52% 单核：PID 22.7%、tracking experiment 11.1%、trajectory 9.3%、tracking
monitor 8.8%。该门在未完成池参数测量时使用 `enable_pool_tracking:=false`；验证机的推进器
模型与池边界确认后默认启动自动跟踪进程，但仲裁、定位和目标新鲜度门仍保持 fail-closed。

`command_authority` 的命令/安全检查保持 100 Hz，Aquaboard 命令心跳为 50 Hz，
重复 UI 状态独立降至 10 Hz。中位边沿立即发布；新鲜度、dead-man 和
150 ms bridge timeout 保持不变。

### 日志隔离

已有一次约 40 分钟运行产生 813,892 条、362 MB JSONL：推进器和仲裁约 100 Hz，轨迹和
PID 约 60 Hz，跟踪状态约 20 Hz。所有记录原先逐条打开、编码、写入、关闭文件。

现在：

- 单一 64 KiB 缓冲句柄，默认每 250 ms flush；abort 和实验阶段事件立即 flush。
- 摘要订阅为 best-effort、depth 1，日志变慢时不会向控制发布者反压或追赶陈旧队列。
- 推进器/轨迹/跟踪上限 20 Hz，仲裁/PID 上限 10 Hz，定位健康保持 1 Hz。
- JSON 使用紧凑分隔符。全频原始数据应由按需 rosbag2 记录，不再伪装成“轻量”JSON。

按已有事件平均尺寸估算，重复摘要由约 340 条/s 降至不高于约 82 条/s，JSON 字节率约
下降 65%–70%。历史 `data/` 已有约 703 MB；本次没有删除任何运行数据。

### 热稳定性

生产栈在 MAXN_SUPER 下的 12 秒现场采样显示 CPU 各核约 18%–43%、GPU 约 7%–34%，
算力并未饱和；但 CPU/Tj 为 98.25/98.38°C。Tj 已超过 95°C active cooling trip，CPU
距离 99°C passive trip 仅约 0.75°C，而 `nvfancontrol` quiet profile 只给出 PWM 181/255。

`robotcore-performance.service` 因此使用 NVIDIA 自带的 `jetson_clocks --fan`：在锁定
CPU/GPU/EMC 时同时停止动态风扇服务并固定 PWM 255。该 unit 明确排在
`nvfancontrol.service` 之后，避免 Jetson 启动阶段的 `nvpower`/动态风扇初始化在稍后覆盖
这些设置。该 unit 必须随部署脚本安装并在安全停机窗口重启后才生效；不要在推进器可能
活动时为验证温度而重启整个 robot stack。

`nvargus-daemon` 和 `zed_x_daemon` 通过 `PartOf=robotcore.service` 与机器人图绑定。
每次重启 RobotCore 都先停止 ZED 客户端，再依次重启 ZED X 和 Argus 后端，最后启动
新的机器人图；不再用 PID 或 IPC socket 采样代替完整的相机生命周期重启。

### ZED 图像质量

生产主题仍维持 ZED odometry 约 30.00 Hz 和 CUDA detections 约 30 Hz，但最近 10 分钟
journal 有 117 条 degraded/noisy-keyframe 警告、3 条 duplicate-frame 和 6 条
`CORRUPTED FRAME` 日志行。先检查照明/曝光、镜头、ZED Link/CSI 连接、供电与上述热状态；
不要设置 `ZED_SDK_GEN3_DISABLE_KEYFRAME_IMAGE_QUALITY_CHECK=ON` 来隐藏告警。修复后应重跑
同样的 10 分钟计数和 30 Hz 抖动测量。

## 不建议立即 C++ 重写的 Python 节点

| 节点 | 结论 | 触发重写的证据 |
|---|---|---|
| `command_authority` | 已迁移到 `robotcore_control_cpp` | C++ 节点以 100 Hz 评估安全状态、50 Hz 发布推进器心跳、10 Hz 发布状态；旧 Python 实现已删除 |
| `pid_controller` | 保留 Python+NumPy/SciPy | 60 Hz callback p99 超过 4 ms，或状态到 PID 命令 p95 超预算 |
| trajectory/tracking/safety | 保留 Python编排 | 明确的 CPU 热点或调度丢期，而不是仅凭语言判断 |
| `run_logger` | 保留独立低优先级 Python 进程 | 缓冲、限频、best-effort 后仍影响控制 trace |
| ONNX policy | 模型大时使用 ONNX Runtime CUDA/TensorRT | 先完成 provider 安装并记录推理 p50/p95/p99；当前生产 launch 未启用 policy |

如果必须继续迁移，优先把 PID 和分配器改为 C++ 组件，再与仲裁节点合成一个
`rclcpp_components` 容器，并保持 PID 专用输入
`/control/pid/thruster_cmd`、中央仲裁和唯一最终输出 `/control/thruster_cmd` 的边界；不要重写
日志、任务管理或 launch Python。

## 已验证与待验证

本机已验证：

- 7 个 ROS 包 Release 构建成功，C++ 使用 `-O3` 和可用时 LTO。
- Python/静态回归 95 项通过。
- C++ 协议、IMU、地图与定位组件测试通过，无失败。
- 隔离 CycloneDDS 域内的合成 VIO+Tag 锚点闭环：
  `BodyState` 共 336 条、59.998 Hz、时间戳严格递增、速度中值 0.2500 m/s。
- 生产 raw Aquaboard IMU：100.00 Hz，累计 192,242 帧时序号缺口、CRC、版本、重复、
  队列溢出均为 0，传输 p95 1.94 ms。
- 生产 ZED odometry 约 30.00 Hz，CUDA AprilTag detections 约 29.85–30.00 Hz。
- 8 核 Cortex-A78AE、15 GiB RAM；`nvpmodel -q` 显示 `MAXN_SUPER`。

受执行沙箱限制，本次不能连接 systemd、ROS DDS 图或 NVIDIA 设备节点，也不能代替真机
满载验收。以下仍以 `LOCALIZATION_CPP_ACCEPTANCE.md` 为准：IMU p95、Tag 融合 p95、
60 Hz 持续频率、CUDA/ZED 满载、故障注入、绝对精度和漂移。

真机验证应同时记录：

```bash
tegrastats --interval 1000
ros2 topic hz /robot/body_state --window 200
ros2 topic hz /control/thruster_cmd --window 200
```

`robotcore_estimator_rate_check.py` 会主动发布合成 VIO/Tag，禁止在生产 DDS
Domain 中运行。只能用隔离 Domain 配合 standalone 融合节点做逻辑回归：

```bash
export ROS_DOMAIN_ID=143 ROS_LOCALHOST_ONLY=1
ros2 run robotcore_sensors vio_tag_fusion_node --ros-args \
  -p output_rate_hz:=60.0
python3 scripts/robotcore_estimator_rate_check.py --duration 8
```

再用 `ros2_tracing` 或等价时间戳探针测量：VIO采样→融合发布、图像时间戳→Tag融合、
BodyState→PID候选→仲裁→bridge dispatch。只有这些 p95/p99 数据能决定是否进入局部 C++
迁移、事件驱动调度或 PREEMPT_RT/实时优先级阶段。
