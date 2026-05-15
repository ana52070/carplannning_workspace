# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a ROS2 workspace for an autonomous mobile robot (Yahboom/CarPlanning chassis). It contains three independent colcon sub-workspaces, each with its own `src/`, `build/`, `install/`, and `log/` directories:

| Directory | Package | Language | Purpose |
|-----------|---------|----------|---------|
| `myslam_car/` | `myslam_car_driver` | Python | Chassis driver + keyboard teleop + static TF |
| `livox/` | `livox_ros_driver2` | C++ | Livox LiDAR driver (HAP / MID360) |
| `rtk_um982/` | `um982_ros2_driver` | Python | UM982/UM980 RTK GPS driver |

Target ROS2 distro: **Humble** (Ubuntu 22.04). Foxy also supported.

---

## Build Commands

Each sub-workspace must be built independently. Source the ROS2 environment first.

```bash
source /opt/ros/humble/setup.bash

# Build chassis driver
cd myslam_car && colcon build && cd ..

# Build Livox LiDAR driver (uses custom build.sh)
cd livox/src/livox_ros_driver2
./build.sh humble      # or ROS2 for Foxy
cd ../../..

# Build RTK GPS driver
cd rtk_um982 && colcon build && cd ..
```

After building, source the install overlay before running nodes:
```bash
source myslam_car/install/setup.bash
source livox/install/setup.bash
source rtk_um982/install/setup.bash
```

---

## Running Nodes

```bash
# Chassis driver (serial port defaults to /dev/ttyUSB0)
ros2 run myslam_car_driver driver_node

# Static TF publisher (IMU, laser, wheel frames)
ros2 run myslam_car_driver robot_static_tf_pub_node

# Keyboard teleop
ros2 run myslam_car_driver keyboard_node

# RTK GPS driver (serial port defaults to /dev/ttyUSB1, baud 115200)
ros2 launch um982_ros2_driver um982_ros2_driver.launch.py port:=/dev/ttyUSB1 baud:=921600

# Livox LiDAR (ROS2, MID360 example)
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

---

## Architecture

### TF Frame Tree

```
odom  ──(dynamic)──▶  base_link  ──(static)──▶  imu_link      (180° Z rotation)
                                 ──(static)──▶  laser         (-90° Z rotation)
                                 ──(static)──▶  base_footprint
                                 ──(static)──▶  front_left/right_wheel
                                 ──(static)──▶  back_left/right_wheel

earth ──(GPS UTM)──▶  base_link  (published by UM982 node)
```

- `odom → base_link`: published dynamically by `carplanning_driver` via wheel odometry dead-reckoning.
- Static transforms: published by `robot_static_tf_pub_node` at startup.

### Published Topics (chassis driver, 10 Hz)

| Topic | Type | Notes |
|-------|------|-------|
| `/odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/imu/data_raw` | `sensor_msgs/Imu` | Raw IMU (no orientation estimate) |
| `/imu/mag` | `sensor_msgs/MagneticField` | Magnetometer |
| `/vel_raw` | `geometry_msgs/Twist` | Encoder-derived velocity in ROS frame |
| `/voltage` | `std_msgs/Float32` | Battery voltage |
| `/joint_states` | `sensor_msgs/JointState` | Wheel joint angles |

### Published Topics (RTK GPS driver, 20 Hz default)

| Topic | Type | Notes |
|-------|------|-------|
| `gps/fix` | `sensor_msgs/NavSatFix` | WGS84 lat/lon/alt with covariance |
| `gps/utmpos` | `nav_msgs/Odometry` | UTM XY position + orientation (from dual-antenna heading) |

### Hardware Coordinate Quirk

The Yahboom Rosmaster chassis has a non-standard axis mapping. The `set_car_motion(vy, vx, vz)` API takes arguments in `(y, x, z)` order, and the physical axes are inverted relative to ROS convention:

```python
# cmd_vel_callback: sends negated and swapped axes to hardware
self.car.set_car_motion(-vy_ros, -vx_ros, angular_ros)

# pub_data: inverse mapping for encoder readback
vx_ros = -vy_hw
vy_ros = -vx_hw
angular_ros = angular_hw
```

Do not change this mapping without re-validating physical motion against RViz display.

### UM982 Serial Protocol

`um982.py` parses three sentence types from the GPS receiver:
- `#PVTSLNA` — position fix (lat/lon/alt + std devs), uses CRC32
- `$GNHPR` — dual-antenna heading/pitch/roll, uses NMEA XOR checksum
- `#BESTNAVA` — velocity, uses CRC32

The serial reader wraps `pyserial` in `io.BufferedReader(buffer_size=4096)` to batch syscalls. The UTM zone transformer is initialized from the first fix and remains fixed for the session.

### Livox LiDAR Driver

- LiDAR IP and network ports are configured in JSON files under `livox/src/livox_ros_driver2/config/`.
- Default host IP assumed to be `192.168.1.5`; LiDAR default `192.168.1.100`.
- `xfer_format` parameter: `0` = Livox PointXYZRTLT, `2` = standard PCL PointXYZI.
- If `liblivox_sdk_shared.so` cannot be found at runtime: `export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib`.

---

## Key Parameters

### Chassis driver (`carplanning_driver.py`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | STM32 serial device |
| `car_type` | `X3` | `X3`/`X1` (diff-drive) or `R2` (mecanum) |
| `imu_link` | `imu_link` | IMU frame ID |
| `xlinear_limit` | `1.0` m/s | Forward velocity clamp |
| `ylinear_limit` | `1.0` m/s | Lateral velocity clamp |
| `angular_limit` | `5.0` rad/s | Angular velocity clamp |

### RTK GPS driver (`um982_ros2_driver`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `/dev/ttyUSB1` | GPS serial device |
| `baud` | `115200` | Baud rate (device may need 921600) |
| `publish_rate` | `20.0` Hz | Publishing frequency |
| `invert_heading` | `false` | Negate heading sign if antenna order is reversed |
