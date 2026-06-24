# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a ROS2 workspace for an autonomous mobile robot (Yahboom/CarPlanning chassis with Jetson Xavier NX). It contains **six** independent colcon sub-workspaces, each with its own `src/`, `build/`, `install/`, and `log/` directories:

| Directory | Package | Language | Purpose |
|-----------|---------|----------|---------|
| `myslam_car/` | `myslam_car_driver` | Python | Chassis driver + keyboard teleop + static TF |
| `livox/` | `livox_ros_driver2` | C++ | Livox LiDAR driver (HAP / MID360) |
| `rtk_um982/` | `um982_ros2_driver` | Python | UM982/UM980 RTK GPS driver |
| `fastlio2/` | `fastlio2`, `hba`, `pgo`, `localizer`, `interface` | C++ | FAST-LIO2 LiDAR-inertial odometry + loop closure + localization |
| `spatio_temporal/` | `spatio_temporal_voxel_layer` | C++ | STVL 3D costmap plugin for Nav2 (OpenVDB-based) |
| `carplanning_code/` | `carplanning_code` | Config-only | Orchestration: launch files, YAML configs, relay/diagnostic scripts |

Target ROS2 distro: **Humble** (Ubuntu 22.04).

> **📖 Detailed architecture:** The `carplanning_code/src/CLAUDE.md` contains in-depth documentation of the navigation stack data flow, node startup order, EKF configuration rationale, and known issues. Read it for any non-trivial work on the navigation pipeline.

---

## Build Commands

Each sub-workspace must be built independently in the correct order (dependencies first). Source the ROS2 environment first.

```bash
source /opt/ros/humble/setup.bash

# 1. Chassis driver (no external deps)
cd myslam_car && colcon build && cd ..

# 2. Livox LiDAR driver (uses custom build.sh)
cd livox/src/livox_ros_driver2
./build.sh humble      # or ROS2 for Foxy
cd ../../..

# 3. RTK GPS driver (no external deps)
cd rtk_um982 && colcon build && cd ..

# 4. Spatio-temporal voxel layer (OpenVDB — slow first build, ~5-10 min)
cd spatio_temporal && colcon build --packages-select spatio_temporal_voxel_layer && cd ..

# 5. FAST-LIO2 (depends on livox_ros_driver2 custom msg interfaces)
source livox/install/setup.bash
cd fastlio2 && colcon build && cd ..

# 6. Carplanning orchestration (depends on spatio_temporal_voxel_layer headers)
source spatio_temporal/install/setup.bash
cd carplanning_code && colcon build --packages-select carplanning_code && cd ..
```

After building, source all install overlays before running nodes:
```bash
source /opt/ros/humble/setup.bash
source myslam_car/install/setup.bash
source livox/install/setup.bash
source rtk_um982/install/setup.bash
source spatio_temporal/install/setup.bash
source fastlio2/install/setup.bash
source carplanning_code/install/setup.bash
```

---

## Running Nodes

### Full bringup (hardware)

```bash
# Start everything: chassis + LiDAR + GPS + FAST-LIO2 + EKF + Nav2
ros2 launch carplanning_code bringup.launch.py

# In a separate terminal: convert Livox CustomMsg → PointCloud2 for Nav2 costmap
# (NOT auto-launched by bringup — must be run manually)
python3 carplanning_code/src/scripts/livox_to_pc2.py
```

### Individual nodes

```bash
# Chassis driver (serial port defaults to /dev/ttyUSB0)
ros2 run myslam_car_driver driver_node

# Static TF publisher (IMU, laser, wheel frames)
ros2 run myslam_car_driver robot_static_tf_pub_node

# Keyboard teleop
ros2 run myslam_car_driver keyboard_node

# RTK GPS driver (serial port defaults to /dev/ttyUSB1)
ros2 launch um982_ros2_driver um982_ros2_driver.launch.py port:=/dev/ttyUSB1 baud:=921600

# Livox LiDAR (ROS2, MID360 example)
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# FAST-LIO2 LiDAR-inertial odometry
ros2 launch fastlio2 lio_launch.py
```

### Utility scripts (in `test_python/`)

```bash
# Drive forward exactly N meters using wheel odometry
python3 test_python/goto.py

# GPS RTK quality monitor — real-time precision grading, drift analysis, CSV logging
python3 test_python/gps_monitor.py --rate 2.0 --csv gps_log.csv
```

---

## Architecture

### Full-system TF tree

```
map ──(EKF: fused LIO + GPS)──► odom ──(chassis wheel odom)──► base_footprint ──► base_link
                                                                                       │
                                                    ┌──────────────────────────────────┤
                                                    │                                  │
                                              livox_frame                            gps
                                           (LiDAR frame,                          (GPS antenna
                                            x=0.14, z=0.26)                       x=-0.14, z=0.12)
```

- `map → odom`: EKF output — fuses FAST-LIO2 velocity + GPS absolute position/yaw
- `odom → base_footprint`: chassis driver publishes this from wheel odometry (NOT fused by EKF)
- `base_link → livox_frame` / `base_link → gps`: static transforms published by bringup launch

### Data flow (simplified)

```
Livox Mid-360 (/livox/lidar, CustomMsg)
    ├──► FAST-LIO2 (tightly coupled with /livox/imu) → /fastlio2/lio_odom
    │         └──► lio_odom_relay.py (injects covariance) → /lio_odom/relay
    │                   └──► EKF ← /odometry/gps ← navsat_transform ← /gps/fix (UM982)
    │                           └──► map → odom TF
    │
    └──► livox_to_pc2.py (format conversion) → /livox/pointcloud2
              └──► STVL 3D costmap layer → Nav2 costmap → Navfn + DWB → /cmd_vel → Chassis
```

The system has **two odometry pipelines**:
- **FAST-LIO2** (primary): LiDAR-inertial odometry, high-frequency (~100 Hz), drift accumulates slowly
- **Chassis wheel odometry** (`/odom`): used only for `odom→base_footprint` TF; NOT fused by EKF (replaced by FAST-LIO2)

GPS provides absolute position/yaw corrections via EKF to bound FAST-LIO2 drift.

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

### Published Topics (FAST-LIO2)

| Topic | Type | Notes |
|-------|------|-------|
| `/fastlio2/lio_odom` | `nav_msgs/Odometry` | LiDAR-inertial odometry, **covariance = 0!** |
| `/fastlio2/registered_cloud` | `sensor_msgs/PointCloud2` | Registered pointcloud in odom frame |

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
- `xfer_format` parameter: `0` = Livox PointXYZRTLT, `1` = Livox CustomMsg (used by FAST-LIO2), `2` = standard PCL PointXYZI.
- If `liblivox_sdk_shared.so` cannot be found at runtime: `export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib`.

### FAST-LIO2

FAST-LIO2 is a computationally efficient LiDAR-inertial odometry system using an iterated extended Kalman filter (IESKF) and an incremental kd-tree (ikd-Tree) for map management. The `fastlio2/` workspace contains five packages:

| Package | Purpose |
|---------|---------|
| `fastlio2` | Core SLAM: IESKF + ikd-Tree, publishes `/fastlio2/lio_odom` |
| `hba` | Hierarchical BA (bundle adjustment) for loop closure with GTSAM |
| `pgo` | Pose graph optimization for loop closure with GTSAM |
| `localizer` | ICP-based global localization against a prior map |
| `interface` | Shared ROS2 msg/srv definitions for inter-package communication |

Key config (`fastlio2/config/lio.yaml`):
- LiDAR topic: `/livox/lidar` (CustomMsg format)
- IMU topic: `/livox/imu`
- `body_frame: base_link`, `world_frame: odom`
- LiDAR extrinsic: `t_il: [-0.011, -0.02329, 0.04412]` (translation from IMU to LiDAR)

### Spatio-Temporal Voxel Layer (STVL)

A Nav2 costmap plugin that uses OpenVDB for efficient 3D volumetric obstacle representation with temporal decay. Observes `/livox/pointcloud2` in `map` frame. Configured via `carplanning_code/src/config/nav2_params.yaml`.

### Carplanning orchestration (`carplanning_code/`)

A configuration-only package that integrates all drivers and the Nav2 stack. See `carplanning_code/src/CLAUDE.md` for full details on:
- Node startup order and timing (with delays)
- EKF configuration rationale (differential mode, GPS absolute corrections)
- navsat_transform setup (UM982 dual-antenna heading)
- Known issues and diagnostic procedures

### Diagnostic Commands

```bash
# EKF real-time probe (topic frequencies, delays, TF health, GPS quality)
python3 carplanning_code/src/scripts/ekf_probe.py --rate 2.0

# One-shot diagnostic snapshot
bash carplanning_code/src/scripts/ekf_quick_check.sh

# Launch all diagnostics together
ros2 launch carplanning_code ekf_diagnostic.launch.py

# GPS RTK quality monitor (real-time precision grading + drift analysis + CSV)
python3 test_python/gps_monitor.py --rate 2.0 --csv gps_log.csv

# Send navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: -1.0, z: 0.0}, \
   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

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

---

## Critical Gotchas

- **`livox_to_pc2.py` must be run separately** — not auto-launched by bringup. Without it, Nav2 costmap sees no obstacles.
- **FAST-LIO2 outputs covariance = 0** — `lio_odom_relay.py` must be running for EKF to properly weight the odometry. Without it, EKF treats LIO as "infinitely precise" and GPS corrections are ignored.
- **`ekf.yaml` `history_length` must be a float** (e.g. `2.0`), not an integer. Using `2` causes EKF to crash with `InvalidParameterTypeException`.
- **Do NOT run Rviz2 on the Xavier NX** — CPU resources are insufficient. Run it remotely on another machine.
- **Build order matters** — `fastlio2` depends on `livox_ros_driver2` custom messages; `carplanning_code` depends on `spatio_temporal_voxel_layer` headers. Source the respective install overlays before building dependents.
- **`use_odometry_yaw: false` in navsat.yaml** — the UM982 dual antennas provide absolute heading; setting this to `true` loses the GPS heading reference and uses EKF's internally propagated yaw.
