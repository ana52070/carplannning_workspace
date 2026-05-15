# CarPlanning 自动驾驶小车工作空间

中文 | [English](./README_EN.md)

基于 ROS2 的自主移动机器人工作空间，底盘采用**亚博智能 X3 差速驱动小车**。整体方案融合了轮式里程计、车载 IMU、Livox 固态激光雷达，以及大唐联芯 UM982 双天线 RTK GPS 模块，实现室外高精度定位。

## 功能包说明

| 子工作空间 | 功能包 | 语言 | 功能 |
|---|---|---|---|
| `myslam_car/` | `myslam_car_driver` | Python | 底盘驱动、静态 TF 发布、键盘遥控 |
| `livox/` | `livox_ros_driver2` | C++ | Livox HAP / MID360 激光雷达驱动 |
| `rtk_um982/` | `um982_ros2_driver` | Python | UM982/UM980 RTK GPS 驱动 |

> **ROS2 版本：** 推荐 Humble（Ubuntu 22.04），也支持 Foxy。

---

## 硬件配置

| 传感器 | 接口 | 默认端口 / IP |
|--------|------|--------------|
| STM32 底盘控制器（亚博 Rosmaster） | 串口 UART | `/dev/ttyUSB0` |
| UM982 RTK GPS | 串口 UART | `/dev/ttyUSB1`，波特率 921600 |
| Livox MID360 激光雷达 | 以太网 | 雷达 `192.168.1.100`，主机 `192.168.1.5` |

---

## 编译

三个子工作空间需要**分别编译**，先 source ROS2 环境。

```bash
source /opt/ros/humble/setup.bash

# 1. 编译底盘驱动
cd myslam_car && colcon build && cd ..

# 2. 编译 Livox 激光雷达驱动（使用专用编译脚本）
cd livox/src/livox_ros_driver2
./build.sh humble        # Foxy 请改为 ROS2
cd ../../..

# 3. 编译 RTK GPS 驱动
cd rtk_um982 && colcon build && cd ..
```

运行节点前需 source 各子工作空间的 install overlay：

```bash
source myslam_car/install/setup.bash
source livox/install/setup.bash
source rtk_um982/install/setup.bash
```

### Livox 前置依赖

Livox 驱动需要先在主机上安装 **Livox-SDK2**，安装方法参见：<https://github.com/Livox-SDK/Livox-SDK2>

若运行时出现 `liblivox_sdk_shared.so: cannot open shared object file`：

```bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
```

---

## 启动运行

### 完整机器人启动流程（推荐顺序）

```bash
# 1. 发布静态 TF（IMU、激光雷达、车轮相对 base_link 的变换）
ros2 run myslam_car_driver robot_static_tf_pub_node

# 2. 底盘驱动——发布 /odom、/imu/data_raw、/imu/mag，广播 odom→base_link TF
ros2 run myslam_car_driver driver_node

# 3. RTK GPS 驱动——发布 gps/fix 和 gps/utmpos
ros2 launch um982_ros2_driver um982_ros2_driver.launch.py \
    port:=/dev/ttyUSB1 baud:=921600

# 4. Livox 激光雷达（以 MID360 为例）
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 键盘遥控

```bash
ros2 run myslam_car_driver keyboard_node
```

按键说明：`i` 前进，`,` 后退，`j`/`l` 旋转，`k`/空格 急停。
`q`/`z` 同步缩放速度，`w`/`x` 仅调线速度，`e`/`c` 仅调角速度。

---

## TF 坐标系树

```
odom ──(动态)──▶ base_link ──(静态)──▶ imu_link        (Z 轴旋转 +180°)
                           ──(静态)──▶ laser            (Z 轴旋转 −90°)
                           ──(静态)──▶ base_footprint
                           ──(静态)──▶ front_left_wheel
                           ──(静态)──▶ front_right_wheel
                           ──(静态)──▶ back_left_wheel
                           ──(静态)──▶ back_right_wheel

earth ──(GPS/UTM)──▶ base_link
```

---

## 主要话题

| 话题 | 消息类型 | 发布节点 | 频率 |
|------|---------|---------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 / 键盘节点 | — |
| `/odom` | `nav_msgs/Odometry` | 底盘驱动 | 10 Hz |
| `/imu/data_raw` | `sensor_msgs/Imu` | 底盘驱动 | 10 Hz |
| `/imu/mag` | `sensor_msgs/MagneticField` | 底盘驱动 | 10 Hz |
| `/vel_raw` | `geometry_msgs/Twist` | 底盘驱动 | 10 Hz |
| `/voltage` | `std_msgs/Float32` | 底盘驱动 | 10 Hz |
| `gps/fix` | `sensor_msgs/NavSatFix` | UM982 驱动 | 20 Hz |
| `gps/utmpos` | `nav_msgs/Odometry` | UM982 驱动 | 20 Hz |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Livox 驱动 | 10 Hz |

---

## 参数配置

### 底盘驱动参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `serial_port` | `/dev/ttyUSB0` | STM32 串口设备路径 |
| `car_type` | `X3` | 车型：`X3`/`X1`（差速）或 `R2`（麦轮） |
| `imu_link` | `imu_link` | IMU 坐标系名称 |
| `xlinear_limit` | `1.0` m/s | 前进速度上限 |
| `ylinear_limit` | `1.0` m/s | 侧移速度上限 |
| `angular_limit` | `5.0` rad/s | 旋转角速度上限 |

### RTK GPS 驱动参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `port` | `/dev/ttyUSB1` | 串口设备路径 |
| `baud` | `115200` | 波特率（实际使用建议 921600） |
| `publish_rate` | `20.0` | 发布频率（Hz） |
| `invert_heading` | `false` | 若双天线安装顺序反置则设为 true |

### Livox 激光雷达配置

编辑 `livox/src/livox_ros_driver2/config/MID360_config.json` 配置雷达 IP 和主机 IP。常用 launch 参数：

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `publish_freq` | `10.0` | 点云发布频率（Hz） |
| `xfer_format` | `0` | `0` = Livox PointXYZRTLT，`2` = PCL PointXYZI |
| `multi_topic` | `0` | `1` = 每台雷达独立话题 |

---

## 依赖安装

```bash
# ROS2 相关包
sudo apt install ros-humble-tf2-ros ros-humble-tf-transformations \
                 ros-humble-nav-msgs ros-humble-sensor-msgs

# Python 包
pip install tf-transformations pyproj pyserial

# 亚博智能底盘库（Rosmaster_Lib）——按亚博官方文档安装
```

---

## 相关链接

- 英文版 README：[README_EN.md](./README_EN.md)
- Livox-SDK2：<https://github.com/Livox-SDK/Livox-SDK2>
- UM982 ROS2 驱动上游：<https://github.com/ironoa/um982_ros2_driver>
