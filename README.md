# fusion_filter_ros2

A **ROS2 IMU filter package** built using the [x-io Technologies Fusion library](https://github.com/xioTechnologies/Fusion).  
This node provides attitude and orientation estimation (AHRS) for IMU data, supporting optional magnetic field input and flexible calibration parameters.

## Features

- Supports gyroscope, accelerometer, and optional magnetometer fusion
- Includes calibration parameters for bias, sensitivity, and misalignment
- Configurable AHRS settings (gain, rejection thresholds, etc.)
- Tested on **ROS2 Jazzy** and **Ubuntu 24.04** with BNO085

## Required
 - `C++20` or higher
 - `rclcpp`
 - `sensor_msgs`

## Build & Installation

```bash
cd your_ros2_workspace/src
git clone --recursive https://github.com/NTques/fusion_filter_ros2.git
cd ..
colcon build --packages-select fusion_filter_ros2
```

##  Launch

```bash
ros2 launch fusion_filter_ros2 fusion_filter.launch.py
```

---

## Topics

| Topic | Type | Direction | Description |
|--------|------|------------|--------------|
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | **Subscribed** | Raw IMU data input |
| `/imu/magnetic_field` | `sensor_msgs/msg/MagneticField` | **Subscribed** | Optional magnetometer input |
| `/imu/data` | `sensor_msgs/msg/Imu` | **Published** | Filtered IMU output with fused orientation |

## Parameters

Below are the parameters available for configuration in `config/fusion_filter.yaml`.

| Name | Type                          | Default                                         | Description |
|------|-------------------------------|-------------------------------------------------|--------------|
| `sample_rate` | `int` | `100`                                           | IMU sample rate in Hz |
| `use_magnetic_field` | `bool` | `false`                                         | Whether to use magnetometer data for heading correction |
| `calibration.gyroscope_misalignment` | `double [9]` | `[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]` | Gyroscope misalignment correction matrix |
| `calibration.gyroscope_sensitivity` | `double [3]` | `[1.0, 1.0, 1.0]`                               | Gyroscope scale correction factor |
| `calibration.gyroscope_offset` | `double [3]` | `[0.0, 0.0, 0.0]`                               | Gyroscope bias (offset) |
| `calibration.accelerometer_misalignment` | `double [9]` | `[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]` | Accelerometer misalignment correction matrix |
| `calibration.accelerometer_sensitivity` | `double [3]` | `[1.0, 1.0, 1.0]`                               | Accelerometer scale correction factor |
| `calibration.accelerometer_offset` | `double [3]` | `[0.0, 0.0, 0.0]`                               | Accelerometer bias (offset) |
| `calibration.soft_iron_matrix` | `double [9]` | `[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]` | Magnetometer soft-iron calibration matrix |
| `calibration.hard_iron_offset` | `double [3]` | `[0.0, 0.0, 0.0]`                               | Magnetometer hard-iron offset vector |
| `ahrs.convention` | `string` | `"ENU"`                                         | Coordinate frame convention (`ENU`, `NED`, `NWU`) |
| `ahrs.gain` | `double` | `0.8`                                           | AHRS proportional gain (fusion responsiveness) |
| `ahrs.gyroscope_range` | `double` | `2000.0`                                        | Gyroscope range in degrees per second |
| `ahrs.acceleration_rejection` | `double` | `10.0`                                          | Threshold for rejecting invalid accelerometer data |
| `ahrs.magnetic_rejection` | `double` | `10.0`                                          | Threshold for rejecting invalid magnetometer data |
| `ahrs.recovery_trigger_period` | `int` | `500`                                           | Time (ms) to trigger recovery from invalid sensor fusion |

## Reference

- [xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion)

