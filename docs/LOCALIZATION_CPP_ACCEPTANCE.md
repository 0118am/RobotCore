# 定位 C++ 重写验收记录

更新时间：2026-08-02。本文把“本机已验证”和“必须在真机执行”严格分开。

## 已完成的软件验证

| 项目 | 结果 | 证据/命令 |
|---|---|---|
| A-board 固件 Release 编译 | PASS | `cmake -S . -B /tmp/acube-cpp-rewrite-build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake -DCMAKE_BUILD_TYPE=Release`；`-O3 + LTO`，ELF/HEX/BIN 生成，Flash 39,292 B，RAM 22,344 B |
| ROS C++ Release + LTO 构建 | PASS | `colcon build --packages-select robotcore_interfaces robotcore_control robotcore_control_cpp robotcore_hardware robotcore_sensors robotcore_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release` |
| launch 静态解析 | PASS | `ros2 launch robotcore_bringup robotcore_edge_system.launch.py --show-args` |
| C++ 协议/时钟单元测试 | PASS | CRC golden vector、frame 4、坏 CRC、倒退时钟 |
| C++ ESKF 单元测试 | PASS | 静止传播、Joseph/PSD、NIS 离群拒绝 |
| 数据链运行语言 | PASS（静态） | UART、IMU 条件化、ZED 适配、Tag 地图 PnP、ESKF、BodyState 均为 C++；launch Python 只做进程编排 |
| 旧 `robot_localization` | PASS（静态） | edge launch 不再启动或依赖该节点，旧 YAML 不参与安装后的运行图 |
| 隔离域合成 ESKF 频率 | PASS（本机动态） | VIO 30 Hz、specific-force IMU 100 Hz、Tag 锚点输入；6 s 内 fused/body 各 336 条、59.998 Hz、时间戳严格递增、速度 0.2500 m/s |

编译器选项为 C++17、Release `-O3` 和可用时 LTO；没有启用
`-ffast-math`。

## 尚未执行，不能标为通过

以下项目需要刷写固件、真实 UART、ZED/Isaac CUDA 满载或独立地面真值。本次代码
实现不能替代这些数据，当前状态均为 **PENDING**。

| 验收项 | 判据 | 状态 |
|---|---|---|
| 固件与主机原子升级 | 主机只接受 frame 4/version 1/CRC 正确数据；推进器物理断开或中位锁定刷写 | PENDING |
| 串口连续样本 | 连续 1,000 个 100 Hz 样本无重复、CRC 错误、序号缺口；MCU 时间严格递增、周期约 10 ms | PENDING |
| 满载 IMU 延迟 | 采样到传播 p95 ≤ 15 ms | PENDING |
| 满载状态延迟/频率 | 状态年龄 p95 ≤ 20 ms；输出 59–61 Hz；无持续队列/内存增长 | PENDING |
| Tag 融合延迟 | 图像观测到融合 p95 ≤ 80 ms | PENDING |
| 绝对精度 | Tag 覆盖区位置 RMSE ≤ 0.10 m、姿态 RMSE ≤ 3°，p95 ≤ 0.15 m/5° | PENDING |
| 静止稳定性 | 60 s 位置 p95 波动 ≤ 0.05 m、速度均值 ≤ 0.02 m/s | PENDING |
| 遮挡漂移 | 30 s 漂移 ≤ 路径长度 2% + 0.05 m，偏航 ≤ 3° | PENDING |
| Tag 重现连续性 | 局部里程计不跳变，全局平滑收敛且控制输入无尖峰 | PENDING |
| Monte Carlo 一致性 | 15 维 NEES 与 3/6 维 NIS 均值落入对应 95% 置信区间 | PENDING |
| 故障注入 | IMU 拔插、版本错、ZED 重启、误 Tag、窗口超时、CUDA 满载均不破坏中位心跳 | PENDING |

## 真机执行顺序

1. 物理断开推进器或确认硬件中位锁定，再刷写
   `/tmp/acube-cpp-rewrite-build/aCube_2023.bin`；固件与 ROS 主机同批切换。
2. 停止 `aboard_bridge_node`，独占串口运行只读工具：
   `python3 scripts/robotcore_aboard_telemetry_probe.py --duration 12`。
3. 检查 frame 4 计数器、MCU tick、CRC、到达间隔，再启动 edge launch。
4. 在 `/diagnostics` 记录 `imu_rate_hz`、`imu_transport_p95_ms`、
   `imu_sequence_gaps`、`imu_queue_high_water`、状态年龄、回放和过期计数。
5. 采集带独立真值的 rosbag，离线对齐时钟后计算 RMSE/p95、NIS/NEES；把数据集、
   commit、参数哈希和数值追加到本文件，不能只填写“目测正常”。

## 当前准确性结论

可以确认的是：软件架构、方程和关键数值保护与计划一致，数据来源及降级语义已经
明确。现在还不能确认“达到 0.10 m/3°”或上述实时 p95 指标，因为尚无新版固件
真机串口记录、满载 Jetson 记录和独立真值数据。只有本页全部 PENDING 项有可复现
证据后，才能完成直接替换验收。
