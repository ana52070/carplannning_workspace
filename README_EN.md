# CarPlanning Workspace

中文文档 | [Chinese](./README.md)

ROS2 workspace for an autonomous mobile robot built on a **Yahboom X3 differential-drive chassis**. The stack integrates wheel odometry, an on-board IMU, a Livox solid-state LiDAR, and a dual-antenna RTK GPS module (Unicorecomm UM982) for outdoor localization.

## Packages

| Sub-workspace | Package | Language | Role |
|---|---|---|---|
| `myslam_car/` | `myslam_car_driver` | Python | Chassis driver, static TF, keyboard teleop |
| `livox/` | `livox_ros_driver2` | C++ | Livox HAP / MID360 LiDAR driver |
| `rtk_um982/` | `um982_ros2_driver` | Python | UM982/UM980 RTK GPS driver |

> **ROS2 version:** Humble (Ubuntu 22.04) recommended. Foxy also supported.

---

## Hardware Setup

| Sensor | Interface | Default Port / IP |
|--------|-----------|-------------------|
| STM32 chassis (Yahboom Rosmaster) | UART | `/dev/ttyUSB0` |
| UM982 RTK GPS | UART | `/dev/ttyUSB1` @ 921600 baud |
| Livox MID360 LiDAR | Ethernet | LiDAR `192.168.1.100`, Host `192.168.1.5` |

---

## Build

Each sub-workspace must be built independently. Source the ROS2 environment first.

```bash
source /opt/ros/humble/setup.bash

# 1. Chassis driver
cd myslam_car && colcon build && cd ..

# 2. Livox LiDAR driver (uses custom build script)
cd livox/src/livox_ros_driver2
./build.sh humble        # use 'ROS2' for Foxy
cd ../../..

# 3. RTK GPS driver
cd rtk_um982 && colcon build && cd ..
```

Source all install overlays before running nodes:

```bash
source myslam_car/install/setup.bash
source livox/install/setup.bash
source rtk_um982/install/setup.bash
```

### Livox dependency

The Livox driver requires **Livox-SDK2** installed on the host:
<https://github.com/Livox-SDK/Livox-SDK2>

If you see `liblivox_sdk_shared.so: cannot open shared object file` at runtime:

```bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
```

---

## Running

### Full robot bringup (recommended order)

```bash
# 1. Static TF (IMU, LiDAR, wheels relative to base_link)
ros2 run myslam_car_driver robot_static_tf_pub_node

# 2. Chassis driver — publishes /odom, /imu/data_raw, /imu/mag, TF odom→base_link
ros2 run myslam_car_driver driver_node

# 3. RTK GPS driver — publishes gps/fix and gps/utmpos
ros2 launch um982_ros2_driver um982_ros2_driver.launch.py \
    port:=/dev/ttyUSB1 baud:=921600

# 4. Livox LiDAR (MID360 example)
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### Keyboard teleoperation

```bash
ros2 run myslam_car_driver keyboard_node
```

Keys: `i` forward, `,` backward, `j`/`l` rotate, `k`/space stop.
`q`/`z` scale all speeds, `w`/`x` linear only, `e`/`c` angular only.

---

## TF Frame Tree

```
odom ──(dynamic)──▶ base_link ──(static)──▶ imu_link        (Z +180°)
                              ──(static)──▶ laser            (Z −90°)
                              ──(static)──▶ base_footprint
                              ──(static)──▶ front_left_wheel
                              ──(static)──▶ front_right_wheel
                              ──(static)──▶ back_left_wheel
                              ──(static)──▶ back_right_wheel

earth ──(GPS/UTM)──▶ base_link
```

---

## Key Topics

| Topic | Message Type | Publisher | Rate |
|-------|-------------|-----------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 / keyboard | — |
| `/odom` | `nav_msgs/Odometry` | chassis driver | 10 Hz |
| `/imu/data_raw` | `sensor_msgs/Imu` | chassis driver | 10 Hz |
| `/imu/mag` | `sensor_msgs/MagneticField` | chassis driver | 10 Hz |
| `/vel_raw` | `geometry_msgs/Twist` | chassis driver | 10 Hz |
| `/voltage` | `std_msgs/Float32` | chassis driver | 10 Hz |
| `gps/fix` | `sensor_msgs/NavSatFix` | UM982 driver | 20 Hz |
| `gps/utmpos` | `nav_msgs/Odometry` | UM982 driver | 20 Hz |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Livox driver | 10 Hz |

---

## Configuration

### Chassis driver parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | STM32 serial device path |
| `car_type` | `X3` | `X3`/`X1` (diff-drive) or `R2` (mecanum) |
| `imu_link` | `imu_link` | IMU frame ID |
| `xlinear_limit` | `1.0` m/s | Forward velocity limit |
| `ylinear_limit` | `1.0` m/s | Lateral velocity limit |
| `angular_limit` | `5.0` rad/s | Rotation rate limit |

### RTK GPS parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `/dev/ttyUSB1` | Serial device |
| `baud` | `115200` | Baud rate |
| `publish_rate` | `20.0` | Hz |
| `invert_heading` | `false` | Negate heading if antennas are swapped |

### Livox LiDAR config

Edit `livox/src/livox_ros_driver2/config/MID360_config.json` to set LiDAR and host IPs. Key launch parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `publish_freq` | `10.0` | Point cloud publish rate (Hz) |
| `xfer_format` | `0` | `0` = Livox PointXYZRTLT, `2` = PCL PointXYZI |
| `multi_topic` | `0` | `1` = separate topic per LiDAR unit |

---

## Dependencies

```bash
# ROS2 packages
sudo apt install ros-humble-tf2-ros ros-humble-tf-transformations \
                 ros-humble-nav-msgs ros-humble-sensor-msgs

# Python packages
pip install tf-transformations pyproj pyserial

# Yahboom chassis library (Rosmaster_Lib) — install per Yahboom documentation
```

---

## Links

- Livox-SDK2: <https://github.com/Livox-SDK/Livox-SDK2>
- UM982 ROS2 driver upstream: <https://github.com/ironoa/um982_ros2_driver>
