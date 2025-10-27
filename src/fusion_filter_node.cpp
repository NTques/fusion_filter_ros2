//
// Created by antique on 10/12/25.
//

#include "fusion_filter_ros2/fusion_filter_node.h"
#include "fusion_filter_ros2/fusion_filter_parameters.hpp"

#define GRAVITY 9.80665

namespace fusion_filter
{
    FusionFilterNode::FusionFilterNode() : Node("imu_fusion_filter"), _steady_clock(RCL_STEADY_TIME)
    {
    }

    void FusionFilterNode::init()
    {
        // declare & get all parameters
        _param_listener = std::make_shared<ParamListener>(this->shared_from_this());
        _params = _param_listener->get_params();

        // print parameters
        auto list_result = this->list_parameters({}, std::numeric_limits<int>::max());
        for (const auto& name : list_result.names)
        {
            RCLCPP_INFO(get_logger(), "\t%s: %s", name.c_str(), this->get_parameter(name).value_to_string().c_str());
        }

        // Fusion AHRS parameters
        _settings.convention = (_params.ahrs.convention == "NED")
                                   ? FusionConventionNed
                                   : (_params.ahrs.convention == "NWU")
                                   ? FusionConventionNwu
                                   : FusionConventionEnu;
        _settings.gain = static_cast<float>(_params.ahrs.gain);
        _settings.gyroscopeRange = static_cast<float>(_params.ahrs.gyroscope_range);
        _settings.accelerationRejection = static_cast<float>(_params.ahrs.acceleration_rejection);
        _settings.magneticRejection = static_cast<float>(_params.ahrs.magnetic_rejection);
        _settings.recoveryTriggerPeriod = _params.ahrs.recovery_trigger_period;

        // Fusion imu calibration parameters
        _gyroscope_misalignment = vector_to_fusion_matrix(_params.calibration.gyroscope.misalignment);
        _gyroscope_sensitivity = vector_to_fusion_vector(_params.calibration.gyroscope.sensitivity);
        _gyroscope_offset = vector_to_fusion_vector(_params.calibration.gyroscope.offset);
        _accelerometer_misalignment = vector_to_fusion_matrix(_params.calibration.accelerometer.misalignment);
        _accelerometer_sensitivity = vector_to_fusion_vector(_params.calibration.accelerometer.sensitivity);
        _accelerometer_offset = vector_to_fusion_vector(_params.calibration.accelerometer.offset);
        _soft_iron_matrix = vector_to_fusion_matrix(_params.calibration.iron.soft_matrix);
        _hard_iron_offset = vector_to_fusion_vector(_params.calibration.iron.hard_offset);

        // filtered IMU data covariances
        _orientation_covariance = vector_to_covariance(_params.orientation_covariance);
        _angular_velocity_covariance = vector_to_covariance(_params.angular_velocity_covariance);
        _linear_acceleration_covariance = vector_to_covariance(_params.linear_acceleration_covariance);

        // initialize fusion ahrs
        FusionOffsetInitialise(&_offset, _params.sample_rate);
        FusionAhrsInitialise(&_ahrs);
        FusionAhrsSetSettings(&_ahrs, &_settings);

        // create publisher & subscription
        rclcpp::QoS qos = rclcpp::SensorDataQoS();
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

        _imu_filtered_pub = create_publisher<sensor_msgs::msg::Imu>("imu/data", qos);
        _imu_sub = create_subscription<sensor_msgs::msg::Imu>(
            "imu/data_raw", qos,
            [this](const sensor_msgs::msg::Imu& msg) { imu_callback(msg); });
        if (_params.use_magnetic_field)
        {
            _mag_sub = create_subscription<sensor_msgs::msg::MagneticField>(
                "imu/magnetic_field", qos,
                [this](const sensor_msgs::msg::MagneticField& msg) { mag_callback(msg); });
        }
    }


    void FusionFilterNode::imu_callback(const sensor_msgs::msg::Imu& msg)
    {
        const auto now = _steady_clock.now();
        if (_prev_time.nanoseconds() == 0)
        {
            _prev_time = now;
            return;
        }

        double dt = (now - _prev_time).seconds();
        _prev_time = now;

        // angular velocity: rad/s -> deg/s
        FusionVector gyro{};
        gyro.axis.x = FusionRadiansToDegrees(static_cast<float>(msg.angular_velocity.x));
        gyro.axis.y = FusionRadiansToDegrees(static_cast<float>(msg.angular_velocity.y));
        gyro.axis.z = FusionRadiansToDegrees(static_cast<float>(msg.angular_velocity.z));

        // linear acceleration: m/s^2 -> g
        FusionVector accel{};
        accel.axis.x = static_cast<float>(msg.linear_acceleration.x / GRAVITY);
        accel.axis.y = static_cast<float>(msg.linear_acceleration.y / GRAVITY);
        accel.axis.z = static_cast<float>(msg.linear_acceleration.z / GRAVITY);

        // apply calibration
        gyro = FusionCalibrationInertial(gyro, _gyroscope_misalignment,
                                         _gyroscope_sensitivity, _gyroscope_offset);
        accel = FusionCalibrationInertial(accel, _accelerometer_misalignment,
                                          _accelerometer_sensitivity, _accelerometer_offset);

        // update gyroscope offset
        if (msg.angular_velocity.x != 0.0 && msg.angular_velocity.y != 0.0 && msg.angular_velocity.z != 0.0)
        {
            gyro = FusionOffsetUpdate(&_offset, gyro);
        }

        // update ahrs
        if (_params.use_magnetic_field)
        {
            FusionVector mag = FusionCalibrationMagnetic(_mag, _soft_iron_matrix, _hard_iron_offset);
            FusionAhrsUpdate(&_ahrs, gyro, accel, mag, static_cast<float>(dt));
        }
        else
        {
            FusionAhrsUpdateNoMagnetometer(&_ahrs, gyro, accel, static_cast<float>(dt));
        }

        FusionVector linear_accel = FusionAhrsGetLinearAcceleration(&_ahrs);
        FusionQuaternion quaternion = FusionAhrsGetQuaternion(&_ahrs);

        // publish filtered imu data
        sensor_msgs::msg::Imu filtered_imu_data;
        filtered_imu_data.header.stamp = msg.header.stamp;
        filtered_imu_data.header.frame_id = msg.header.frame_id;

        filtered_imu_data.orientation.x = quaternion.element.x;
        filtered_imu_data.orientation.y = quaternion.element.y;
        filtered_imu_data.orientation.z = quaternion.element.z;
        filtered_imu_data.orientation.w = quaternion.element.w;

        filtered_imu_data.angular_velocity.x = FusionDegreesToRadians(gyro.axis.x);
        filtered_imu_data.angular_velocity.y = FusionDegreesToRadians(gyro.axis.y);
        filtered_imu_data.angular_velocity.z = FusionDegreesToRadians(gyro.axis.z);

        filtered_imu_data.linear_acceleration.x = linear_accel.axis.x * GRAVITY;
        filtered_imu_data.linear_acceleration.y = linear_accel.axis.y * GRAVITY;
        filtered_imu_data.linear_acceleration.z = linear_accel.axis.z * GRAVITY;

        filtered_imu_data.orientation_covariance = _orientation_covariance;
        filtered_imu_data.angular_velocity_covariance = _angular_velocity_covariance;
        filtered_imu_data.linear_acceleration_covariance = _linear_acceleration_covariance;

        _imu_filtered_pub->publish(filtered_imu_data);
    }

    void FusionFilterNode::mag_callback(const sensor_msgs::msg::MagneticField& mag)
    {
        _mag.axis.x = static_cast<float>(mag.magnetic_field.x);
        _mag.axis.y = static_cast<float>(mag.magnetic_field.y);
        _mag.axis.z = static_cast<float>(mag.magnetic_field.z);
    }

    std::array<double, 9> FusionFilterNode::vector_to_covariance(const std::vector<double>& vec)
    {
        std::array<double, 9> covariance{};
        if (vec.size() != 9)
        {
            RCLCPP_ERROR(get_logger(), "Input vector size must be 9 for covariance conversion.");
            return covariance;
        }

        std::ranges::copy(vec, covariance.data());

        return covariance;
    }

    FusionVector FusionFilterNode::vector_to_fusion_vector(const std::vector<double>& vec)
    {
        FusionVector fusion_vector{};
        if (vec.size() != 3)
        {
            RCLCPP_ERROR(get_logger(), "Input vector size must be 3 for FusionVector conversion.");
            return fusion_vector;
        }

        fusion_vector.array[0] = vec[0];
        fusion_vector.array[1] = vec[1];
        fusion_vector.array[2] = vec[2];

        return fusion_vector;
    }

    FusionMatrix FusionFilterNode::vector_to_fusion_matrix(const std::vector<double>& vec)
    {
        FusionMatrix fusion_matrix;
        if (vec.size() != 9)
        {
            RCLCPP_ERROR(get_logger(), "Input vector size must be 9 for FusionMatrix conversion.");
            return fusion_matrix;
        }

        fusion_matrix.array[0][0] = vec[0];
        fusion_matrix.array[0][1] = vec[1];
        fusion_matrix.array[0][2] = vec[2];
        fusion_matrix.array[1][0] = vec[3];
        fusion_matrix.array[1][1] = vec[4];
        fusion_matrix.array[1][2] = vec[5];
        fusion_matrix.array[2][0] = vec[6];
        fusion_matrix.array[2][1] = vec[7];
        fusion_matrix.array[2][2] = vec[8];

        return fusion_matrix;
    }
} // fusion
