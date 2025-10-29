# fusion_filter_ros2

A **ROS2 IMU filter package** built using
the [x-io Technologies Fusion library](https://github.com/xioTechnologies/Fusion).  
This node provides attitude and orientation estimation (AHRS) for IMU data, supporting optional magnetic field input and
flexible calibration parameters.

## Features

- Supports gyroscope, accelerometer, and optional magnetometer fusion
- Includes calibration parameters for bias, sensitivity, and misalignment
- Configurable AHRS settings (gain, rejection thresholds, etc.)
- Tested on **ROS2 Jazzy** and **Ubuntu 24.04** with **BNO085**

## Required

- `C++20` or higher
- `rclcpp`
- `sensor_msgs`
- `generate_parameter_library`

## Build & Installation

```bash
cd your_ros2_workspace/src
git clone --recursive https://github.com/NTques/fusion_filter_ros2.git
cd ..
colcon build --packages-select fusion_filter_ros2
```

## Launch

```bash
ros2 launch fusion_filter_ros2 fusion_filter.launch.py
```

---

## Topics

| Topic                 | Type                            | Direction      | Description                                |
|-----------------------|---------------------------------|----------------|--------------------------------------------|
| `/imu/data_raw`       | `sensor_msgs/msg/Imu`           | **Subscribed** | Raw IMU data input                         |
| `/imu/magnetic_field` | `sensor_msgs/msg/MagneticField` | **Subscribed** | Optional magnetometer input                |
| `/imu/data`           | `sensor_msgs/msg/Imu`           | **Published**  | Filtered IMU output with fused orientation |

## Parameters

Below are the parameters available for configuration in `config/fusion_filter.yaml`.

| Name                                     | Type         | Default                                         | Description                                                                                                                                                                                                                                                |
|------------------------------------------|--------------|-------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `sample_rate`                            | `int`        | `100`                                           | IMU sample rate in Hz                                                                                                                                                                                                                                      |
| `use_magnetic_field`                     | `bool`       | `false`                                         | Whether to use magnetometer data for heading correction                                                                                                                                                                                                    |
| `orientation_covariance`                 | `double [9]` | `[0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]` | Orientation covariance of the filtered IMU data                                                                                                                                                                                                            |
| `angular_velocity_covariance`            | `double [9]` | `[0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]` | Angular velocity covariance of the filtered IMU data                                                                                                                                                                                                       |
| `linear_acceleration_covariance`         | `double [9]` | `[0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]` | Linear acceleration covariance of the filtered IMU data                                                                                                                                                                                                    |
| `calibration.gyroscope.misalignment`     | `double [9]` | `[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]` | Gyroscope misalignment correction matrix                                                                                                                                                                                                                   |
| `calibration.gyroscope.sensitivity`      | `double [3]` | `[1.0, 1.0, 1.0]`                               | Gyroscope scale correction factor                                                                                                                                                                                                                          |
| `calibration.gyroscope.offset`           | `double [3]` | `[0.0, 0.0, 0.0]`                               | Gyroscope bias (offset)                                                                                                                                                                                                                                    |
| `calibration.accelerometer.misalignment` | `double [9]` | `[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]` | Accelerometer misalignment correction matrix                                                                                                                                                                                                               |
| `calibration.accelerometer.sensitivity`  | `double [3]` | `[1.0, 1.0, 1.0]`                               | Accelerometer scale correction factor                                                                                                                                                                                                                      |
| `calibration.accelerometer.offset`       | `double [3]` | `[0.0, 0.0, 0.0]`                               | Accelerometer bias (offset)                                                                                                                                                                                                                                |
| `calibration.iron.soft_matrix`           | `double [9]` | `[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]` | Magnetometer soft-iron calibration matrix                                                                                                                                                                                                                  |
| `calibration.iron.hard_offset`           | `double [3]` | `[0.0, 0.0, 0.0]`                               | Magnetometer hard-iron offset vector                                                                                                                                                                                                                       |
| `ahrs.convention`                        | `string`     | `"ENU"`                                         | Earth axes convention (NWU, ENU, or NED)                                                                                                                                                                                                                   |
| `ahrs.gain`                              | `double`     | `0.8`                                           | Determines the influence of the gyroscope relative to other sensors. A value of zero will disable initialisation and the acceleration and magnetic rejection features. A value of 0.5 is appropriate for most applications.                                |
| `ahrs.gyroscope_range`                   | `double`     | `2000.0`                                        | Gyroscope range (in degrees per second). Angular rate recovery will activate if the gyroscope measurement exceeds 98% of this value. A value of zero will disable this feature. The value should be set to the range specified in the gyroscope datasheet. |
| `ahrs.acceleration_rejection`            | `double`     | `10.0`                                          | Threshold (in degrees) used by the acceleration rejection feature. A value of zero will disable this feature. A value of 10 degrees is appropriate for most applications.                                                                                  |
| `ahrs.magnetic_rejection`                | `double`     | `10.0`                                          | 	Threshold (in degrees) used by the magnetic rejection feature. A value of zero will disable the feature. A value of 10 degrees is appropriate for most applications.                                                                                      |
| `ahrs.recovery_trigger_period`           | `int`        | `500`                                           | Acceleration and magnetic recovery trigger period (in samples). A value of zero will disable the acceleration and magnetic rejection features. A period of 5 seconds is appropriate for most applications.                                                 |

## Reference

- [xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion)

