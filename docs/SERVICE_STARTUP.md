# RobotCore 生产服务启动命令

RobotCore 数据链和网页分别由 `robotcore.service`、
`control-interface.service` 监管，`robotcore-stack.target` 用于一次启动或停止
两者。定位进程固定到 CPU 2–7，网页固定到 CPU 0–1；启动 RobotCore 前会执行
Jetson `MAXN_SUPER` mode 0 和 `jetson_clocks`。请保证散热和供电满足最大功耗。

## 首次安装或更新 unit

推荐在 RobotCore 仓库根目录执行一次安装脚本。它安装 Cyclone DDS、unit、DDS
XML 和 sysctl，并把 service 指向当前已经构建的 C++ 工作空间；为了推进器安全
不会自动启动：

```bash
sudo bash scripts/install_robotcore_services.sh
```

当前 Jetson 上这条命令等价于显式指定：

```bash
sudo bash scripts/install_robotcore_services.sh \
  --robot-workspace /home/nvidia/RobotCore/ros_ws \
  --web-workspace /home/nvidia/ControlInterface \
  --zed-workspace /home/nvidia/ros2_ws
```

脚本会在安装前确认三个工作空间已经构建，并检查 C++
`vio_tag_fusion_node`。因此不会误用 `/opt/RobotCore` 中遗留的 Python 安装。
因为 `/home/nvidia` 默认不可由服务账户穿过，脚本只给 `robotcore` 增加该目录的
路径穿越 ACL，不授予列目录或写入权限。若正式版本部署到 `/opt`，用上面的参数
显式改成对应 `/opt` 路径即可。

等价的手动步骤如下：

```bash
sudo install -m 0644 host_manager/systemd/robotcore.service /etc/systemd/system/
sudo install -m 0644 host_manager/systemd/control-interface.service /etc/systemd/system/
sudo install -m 0644 host_manager/systemd/robotcore-host-manager.service /etc/systemd/system/
sudo install -m 0644 host_manager/systemd/robotcore-performance.service /etc/systemd/system/
sudo install -m 0644 host_manager/systemd/robotcore-stack.target /etc/systemd/system/
sudo install -m 0644 host_manager/config/cyclonedds.xml /etc/robotcore/cyclonedds.xml
sudo install -m 0644 host_manager/systemd/99-robotcore-dds.conf /etc/sysctl.d/99-robotcore-dds.conf
sudo rm -f /etc/systemd/system/robotcore.service.d/tag-vio.conf
sudo install -m 0644 host_manager/systemd/robotcore-argus.conf /etc/systemd/system/robotcore.service.d/argus.conf
sudo systemctl daemon-reload
sudo sysctl -p /etc/sysctl.d/99-robotcore-dds.conf
sudo systemctl disable robotcore.service control-interface.service
sudo systemctl enable robotcore-stack.target
```

按当前工作目录安装后，检查 `/etc/robotcore/edge.env` 至少包含：

```text
ROBOTCORE_WORKSPACE=/home/nvidia/RobotCore/ros_ws
CONTROL_INTERFACE_WORKSPACE=/home/nvidia/ControlInterface
ZED_WORKSPACE=/home/nvidia/ros2_ws
ROBOTCORE_ABOARD_PORT=/dev/robotcore/aboard
ROBOTCORE_APRILTAG_MAP_FILE=/etc/robotcore/apriltag_map.json
CONTROL_INTERFACE_HOST=127.0.0.1
CONTROL_INTERFACE_PORT=8080
ROS_LOCALHOST_ONLY=1
ROS_DOMAIN_ID=42
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///etc/robotcore/cyclonedds.xml
ROBOTCORE_RUN_ROOT=/var/lib/robotcore/runs
```

首次还必须安装 Cyclone DDS：

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-rmw-cyclonedds-cpp
```

RobotCore 与网页必须使用相同的 `rmw_cyclonedds_cpp`、
`CYCLONEDDS_URI`、`ROS_DOMAIN_ID` 和 `ROS_LOCALHOST_ONLY`。unit 在 Cyclone
库或 XML 缺失时会拒绝启动，不会自动回退到 Fast DDS。

配置 XML 已用 Cyclone DDS 0.10.5 的 `ddsperf` 做过 1 秒本机回环验证：约
853.35 ksample/s、27.31 Mbit/s、丢包 0。这个结果证明配置可解析且回环吞吐充足，不代替完整
ROS/CUDA 满载延迟验收。

修改环境文件后执行 `sudo systemctl restart robotcore-stack.target`。当前通过
`ROS_LOCALHOST_ONLY=1` 让 Cyclone 使用回环接口，以获得最低本机抖动；XML 不再
重复声明 `lo`，否则 `rmw_cyclonedds_cpp` 会拒绝创建 Domain。网页仍可通过 HTTP
暴露到局域网。
若需要远程 ROS 2 DDS 调试，除了把 `ROS_LOCALHOST_ONLY` 改为 `0`，还必须另建
使用实际网卡的 Cyclone XML，并让远程终端使用同一个 `ROS_DOMAIN_ID`。

## 每次使用的命令

一次启动 RobotCore 和网页：

```bash
sudo systemctl start robotcore-stack.target
```

设置开机启动并立即启动：

```bash
sudo systemctl enable --now robotcore-stack.target
```

一次停止两者：

```bash
sudo systemctl stop robotcore-stack.target
```

一次重启两者：

```bash
sudo systemctl restart robotcore-stack.target
```

只重启机器人链路，不重启网页：

```bash
sudo systemctl restart robotcore.service
```

只重启网页：

```bash
sudo systemctl restart control-interface.service
```

查看总体状态：

```bash
systemctl status robotcore-stack.target robotcore.service control-interface.service --no-pager
```

持续查看 RobotCore 日志：

```bash
journalctl -fu robotcore.service
```

持续查看网页日志：

```bash
journalctl -fu control-interface.service
```

同时查看两边日志：

```bash
journalctl -fu robotcore.service -u control-interface.service
```

默认网页地址是 `http://127.0.0.1:8080`。需要局域网访问时，把
`CONTROL_INTERFACE_HOST` 改成 `0.0.0.0`，重启网页服务，并使用 Jetson 的 IP。

## 启动前检查

推进器必须断开或确认中位锁定，然后执行：

```bash
test -e /dev/robotcore/aboard
test -r /etc/robotcore/apriltag_map.json
nvpmodel -q
sudo systemd-analyze verify /etc/systemd/system/robotcore*.service /etc/systemd/system/control-interface.service /etc/systemd/system/robotcore-stack.target
```

服务启动后检查关键频率：

```bash
source /home/nvidia/RobotCore/scripts/robotcore_ros_env.sh
ros2 topic hz /sensors/external_imu
ros2 topic hz /zedx/zed_node/odom
ros2 topic hz /localization/apriltag/detections
ros2 topic hz /robot/body_state
ros2 topic echo /diagnostics --once
```
