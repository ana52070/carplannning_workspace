# CarPlanning 自动驾驶小车工作空间

中文 | [English](./README_EN.md)

基于 ROS2 Humble 的自主移动机器人工作空间，底盘采用**亚博智能 X3 差速驱动小车**（Jetson Xavier NX），融合了 FAST-LIO2 激光惯性里程计、Livox Mid-360 固态激光雷达、大唐联芯 UM982 双天线 RTK GPS、以及 Nav2 导航框架，实现室外高精度定位与无图导航。

## 仓库结构

本仓库使用 **git 子模块（submodule）** 管理各功能包：

```
carplannning_workspace/              # 主仓库
├── carplanning_code/src/            # 子模块: 导航启动与配置
├── rtk_um982/src/um982_ros2_driver/ # 子模块: UM982 GPS 驱动
├── fastlio2/src/FASTLIO2_ROS2/      # 子模块: FAST-LIO2 激光惯性里程计
├── myslam_car/src/                  # 内联: 亚博底盘驱动
├── livox/src/                       # 内联: Livox 激光雷达驱动
└── spatio_temporal/src/             # 内联: STVL 3D 代价地图插件
```

```bash
# 克隆时需一并拉取子模块
git clone --recurse-submodules https://github.com/ana52070/carplannning_workspace.git
```

## 功能包说明

| 子工作空间 | 功能包 | 语言 | 功能 |
|-----------|--------|------|------|
| `myslam_car/` | `myslam_car_driver` | Python | 底盘驱动、静态 TF 发布、键盘遥控 |
| `livox/` | `livox_ros_driver2` | C++ | Livox HAP / MID360 激光雷达驱动 |
| `rtk_um982/` | `um982_ros2_driver` | Python | UM982 RTK GPS 驱动（子模块） |
| `fastlio2/` | `fastlio2` + `hba` + `pgo` + `localizer` | C++ | FAST-LIO2 激光惯性里程计 + 回环检测 + 重定位（子模块） |
| `spatio_temporal/` | `spatio_temporal_voxel_layer` | C++ | OpenVDB 3D 代价地图插件 (STVL) |
| `carplanning_code/` | `carplanning_code` | 配置 | 导航启动文件、EKF 配置、Nav2 参数（子模块） |

> **ROS2 版本：** Humble（Ubuntu 22.04）

---

## 硬件配置

| 硬件 | 接口 | 默认端口 / IP |
|------|------|--------------|
| 计算平台 | Jetson Xavier NX | — |
| STM32 底盘控制器 | 串口 UART | `/dev/ttyUSB0` |
| UM982 RTK GPS | 串口 UART | `/dev/ttyUSB1`，波特率 921600 |
| Livox MID360 激光雷达 | 以太网 | 雷达 `192.168.1.100`，主机 `192.168.1.5` |

---

## 编译

六个子工作空间需**按依赖顺序分别编译**：

```bash
source /opt/ros/humble/setup.bash

# 1. 底盘驱动（无外部依赖）
cd myslam_car && colcon build && cd ..

# 2. Livox 激光雷达驱动（使用专用编译脚本）
cd livox/src/livox_ros_driver2
./build.sh humble
cd ../../..

# 3. RTK GPS 驱动（无外部依赖）
cd rtk_um982 && colcon build && cd ..

# 4. STVL 代价地图插件（依赖 OpenVDB，首次编译较慢 ~5-10 分钟）
cd spatio_temporal && colcon build --packages-select spatio_temporal_voxel_layer && cd ..

# 5. FAST-LIO2（依赖 livox_ros_driver2 自定义消息）
source livox/install/setup.bash
cd fastlio2 && colcon build && cd ..

# 6. 导航启动包（依赖 spatio_temporal_voxel_layer 头文件）
source spatio_temporal/install/setup.bash
cd carplanning_code && colcon build --packages-select carplanning_code && cd ..
```

### Livox 前置依赖

Livox 驱动需要先安装 **Livox-SDK2**：<https://github.com/Livox-SDK/Livox-SDK2>

若运行时出现 `liblivox_sdk_shared.so: cannot open shared object file`：

```bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
```

---

## 启动运行

### 完整 bringup（推荐）

```bash
# 1. 先 source 所有工作空间
source /opt/ros/humble/setup.bash
source myslam_car/install/setup.bash
source livox/install/setup.bash
source rtk_um982/install/setup.bash
source spatio_temporal/install/setup.bash
source fastlio2/install/setup.bash
source carplanning_code/install/setup.bash

# 2. 一键启动（底盘 + 雷达 + GPS + FAST-LIO2 + EKF + Nav2）
ros2 launch carplanning_code bringup.launch.py

# 3. 另开终端：Livox CustomMsg → PointCloud2 转换（Nav2 代价地图需要）
python3 carplanning_code/src/scripts/livox_to_pc2.py
```

### 单独启动各节点

```bash
# 底盘驱动
ros2 run myslam_car_driver driver_node

# 静态 TF 发布
ros2 run myslam_car_driver robot_static_tf_pub_node

# RTK GPS
ros2 launch um982_ros2_driver um982_ros2_driver.launch.py port:=/dev/ttyUSB1 baud:=921600

# Livox 激光雷达 (MID360)
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# FAST-LIO2
ros2 launch fastlio2 lio_launch.py
```

### 键盘遥控

```bash
ros2 run myslam_car_driver keyboard_node
```

按键：`i` 前进，`,` 后退，`j`/`l` 旋转，`k`/空格 急停。`q`/`z` 缩放速度，`w`/`x` 调线速度，`e`/`c` 调角速度。

---

## 系统架构

### 数据流

```
Livox Mid-360 (/livox/lidar, CustomMsg)
    ├──► FAST-LIO2 ──► /fastlio2/lio_odom ──► lio_odom_relay ──► EKF
    │                                                              ↑
    │         UM982 (/gps/fix) ──► navsat_transform ──► /odometry/gps
    │                                                              │
    └──► livox_to_pc2 ──► /livox/pointcloud2 ──► STVL 3D 代价地图 ──► Nav2 ──► /cmd_vel ──► 底盘
```

### TF 坐标系树

```
map ──(EKF)──► odom ──(底盘)──► base_footprint ──► base_link
                                                       │
                            ┌──────────────────────────┤
                            │                          │
                      livox_frame                    gps
                   (LiDAR, z=0.26)             (天线, z=0.12)
```

### 主要话题

| 话题 | 消息类型 | 发布节点 | 频率 |
|------|---------|---------|------|
| `/odom` | `nav_msgs/Odometry` | 底盘驱动 | 10 Hz |
| `/imu/data_raw` | `sensor_msgs/Imu` | 底盘驱动 | 10 Hz |
| `/imu/mag` | `sensor_msgs/MagneticField` | 底盘驱动 | 10 Hz |
| `/vel_raw` | `geometry_msgs/Twist` | 底盘驱动 | 10 Hz |
| `/voltage` | `std_msgs/Float32` | 底盘驱动 | 10 Hz |
| `gps/fix` | `sensor_msgs/NavSatFix` | UM982 驱动 | 20 Hz |
| `gps/utmpos` | `nav_msgs/Odometry` | UM982 驱动 | 20 Hz |
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | Livox 驱动 | 10 Hz |
| `/livox/pointcloud2` | `sensor_msgs/PointCloud2` | livox_to_pc2 | 10 Hz |
| `/fastlio2/lio_odom` | `nav_msgs/Odometry` | FAST-LIO2 | ~33 Hz |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 | — |

---

## 参数配置

### 底盘驱动参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `serial_port` | `/dev/ttyUSB0` | STM32 串口设备 |
| `car_type` | `X3` | 车型：`X3`/`X1`（差速）或 `R2`（麦轮） |
| `xlinear_limit` | `1.0` m/s | 前进速度上限 |
| `angular_limit` | `5.0` rad/s | 旋转角速度上限 |

### RTK GPS 驱动参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `port` | `/dev/ttyUSB1` | 串口设备 |
| `baud` | `921600` | 波特率 |
| `publish_rate` | `20.0` Hz | 发布频率 |
| `invert_heading` | `false` | 双天线反序时设为 true |

### FAST-LIO2 关键参数（Xavier NX 优化）

| 参数 | 值 | 说明 |
|------|-----|------|
| `lidar_filter_num` | 10 | 每 10 点取 1，降低输入 |
| `scan_resolution` | 0.3 | 体素降采样 |
| `cube_len` | 100 m | 地图边界框 |
| `ieskf_max_iter` | 3 | 最大迭代次数 |
| 定时器 | 30 ms | ~33 Hz 主循环 |

---

## 诊断工具

```bash
# EKF 实时诊断
python3 carplanning_code/src/scripts/ekf_probe.py --rate 2.0

# GPS RTK 质量监控（精度分级 + 漂移分析 + CSV 记录）
python3 test_python/gps_monitor.py --rate 2.0 --csv gps_log.csv

# 发送导航目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: -1.0, z: 0.0}, \
   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

---

## 相关链接

- [carplanning_code](https://github.com/ana52070/carplanning_code) — 导航启动与配置
- [um982_ros2_humble_driver](https://github.com/ana52070/um982_ros2_humble_driver) — UM982 GPS 驱动
- [FASTLIO2_ROS2 (fork)](https://github.com/ana52070/FASTLIO2_ROS2) — 激光惯性里程计
- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
- [STVL](https://github.com/SteveMacenski/spatio_temporal_voxel_layer)
