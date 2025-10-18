//
// Created by antique on 10/12/25.
//

#include "fusion_ros2/fusion_filter_node.h"


#define GRAVITY 9.80665
#define FUSION_VECTOR_ZERO_STD_VECTOR (std::vector<double>{0.0, 0.0, 0.0})
#define FUSION_VECTOR_ONES_STD_VECTOR (std::vector<double>{1.0, 1.0, 1.0})
#define FUSION_IDENTITY_MATRIX_STD_VECTOR (std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0})

namespace fusion
{
    FusionFilterNode::FusionFilterNode() : Node("imu_fusion_filter"), _steady_clock(RCL_STEADY_TIME)
    {
        // common parameters
        declare_parameter("sample_rate", 100);
        declare_parameter("use_magnetic_field", false);

        get_parameter("sample_rate", _sample_rate);
        get_parameter("use_magnetic_field", _use_magnetic_field);

        // calibration parameters
        _gyroscope_misalignment = get_calibration_parameter_as_fusion_matrix(
            "gyroscope_misalignment", FUSION_IDENTITY_MATRIX_STD_VECTOR);
        _gyroscope_sensitivity = get_calibration_parameter_as_fusion_vector(
            "gyroscope_sensitivity", FUSION_VECTOR_ONES_STD_VECTOR);
        _gyroscope_offset = get_calibration_parameter_as_fusion_vector(
            "gyroscope_offset", FUSION_VECTOR_ZERO_STD_VECTOR);
        _accelerometer_misalignment = get_calibration_parameter_as_fusion_matrix(
            "accelerometer_misalignment", FUSION_IDENTITY_MATRIX_STD_VECTOR);
        _accelerometer_sensitivity = get_calibration_parameter_as_fusion_vector(
            "accelerometer_sensitivity", FUSION_VECTOR_ONES_STD_VECTOR);
        _accelerometer_offset = get_calibration_parameter_as_fusion_vector(
            "accelerometer_offset", FUSION_VECTOR_ZERO_STD_VECTOR);
        _soft_iron_matrix = get_calibration_parameter_as_fusion_matrix(
            "soft_iron_matrix", FUSION_IDENTITY_MATRIX_STD_VECTOR);
        _hard_iron_offset = get_calibration_parameter_as_fusion_vector(
            "hard_iron_offset", FUSION_VECTOR_ZERO_STD_VECTOR);

        // AHRS parameters
        this->declare_parameter("ahrs.convention", "ENU");
        this->declare_parameter("ahrs.gain", 0.8);
        this->declare_parameter("ahrs.gyroscope_range", 2000.0);
        this->declare_parameter("ahrs.acceleration_rejection", 10.0);
        this->declare_parameter("ahrs.magnetic_rejection", 10.0);
        this->declare_parameter("ahrs.recovery_trigger_period", 500);

        std::string convention;
        this->get_parameter("ahrs.convention", convention);
        this->get_parameter("ahrs.gain", _settings.gain);
        this->get_parameter("ahrs.gyroscope_range", _settings.gyroscopeRange);
        this->get_parameter("ahrs.acceleration_rejection", _settings.accelerationRejection);
        this->get_parameter("ahrs.magnetic_rejection", _settings.magneticRejection);
        this->get_parameter("ahrs.recovery_trigger_period", _settings.recoveryTriggerPeriod);

        if (convention == "NED") _settings.convention = FusionConventionNed;
        else if (convention == "NWU") _settings.convention = FusionConventionNwu;
        else _settings.convention = FusionConventionEnu;

        // Print parameter values
        RCLCPP_INFO(this->get_logger(), "================= IMU Fusion Filter Parameters =================");
        auto list_result = this->list_parameters({}, std::numeric_limits<int>::max());
        for (const auto& name : list_result.names)
        {
            if (name.find("./") != std::string::npos) continue; // pass inner parameters
            RCLCPP_INFO(get_logger(), "\t%s: %s", name.c_str(), this->get_parameter(name).value_to_string().c_str());
        }
        RCLCPP_INFO(this->get_logger(), "================================================================");

        // initialize Fusion AHRS
        FusionOffsetInitialise(&_offset, _sample_rate);
        FusionAhrsInitialise(&_ahrs);
        FusionAhrsSetSettings(&_ahrs, &_settings);

        // create publisher & subscription
        _imu_filtered_pub = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());
        _imu_sub = create_subscription<sensor_msgs::msg::Imu>(
            "imu/data_raw", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu& msg) { imu_callback(msg); });
        if (_use_magnetic_field)
        {
            _mag_sub = create_subscription<sensor_msgs::msg::MagneticField>(
                "imu/magnetic_field", rclcpp::SensorDataQoS(),
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
        if (dt <= 0.0)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Unusual dt detected: %.6f s — replacing with 1/sample_rate", dt);
            dt = 1.0 / static_cast<double>(_sample_rate);
        }

        // Angular velocity: rad/s -> deg/s
        FusionVector gyro{};
        gyro.axis.x = FusionRadiansToDegrees(static_cast<float>(msg.angular_velocity.x));
        gyro.axis.y = FusionRadiansToDegrees(static_cast<float>(msg.angular_velocity.y));
        gyro.axis.z = FusionRadiansToDegrees(static_cast<float>(msg.angular_velocity.z));

        // Linear acceleration: m/s^2 -> g
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
        if (_use_magnetic_field)
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

        _imu_filtered_pub->publish(filtered_imu_data);
    }

    void FusionFilterNode::mag_callback(const sensor_msgs::msg::MagneticField& mag)
    {
        _mag.axis.x = static_cast<float>(mag.magnetic_field.x);
        _mag.axis.y = static_cast<float>(mag.magnetic_field.y);
        _mag.axis.z = static_cast<float>(mag.magnetic_field.z);
    }

    FusionVector FusionFilterNode::get_calibration_parameter_as_fusion_vector(const std::string& name,
                                                                              const std::vector<double>& default_value)
    {
        this->declare_parameter("calibration." + name, default_value);
        std::vector<double> vector = get_parameter("calibration." + name).as_double_array();

        FusionVector output = FUSION_VECTOR_ZERO;
        if (vector.size() != 3)
        {
            RCLCPP_ERROR(get_logger(), "Parameter '%s' must have 3 elements!", name.c_str());
            return output;
        }

        output.array[0] = static_cast<float>(vector[0]);
        output.array[1] = static_cast<float>(vector[1]);
        output.array[2] = static_cast<float>(vector[2]);

        return output;
    }

    FusionMatrix FusionFilterNode::get_calibration_parameter_as_fusion_matrix(const std::string& name,
                                                                              const std::vector<double>& default_value)
    {
        this->declare_parameter("calibration." + name, default_value);
        std::vector<double> vector = get_parameter("calibration." + name).as_double_array();

        FusionMatrix output = FUSION_IDENTITY_MATRIX;
        if (vector.size() != 9)
        {
            RCLCPP_ERROR(get_logger(), "Parameter '%s' must have 9 elements!", name.c_str());
            return output;
        }

        output.array[0][0] = static_cast<float>(vector[0]);
        output.array[0][1] = static_cast<float>(vector[1]);
        output.array[0][2] = static_cast<float>(vector[2]);
        output.array[1][0] = static_cast<float>(vector[3]);
        output.array[1][1] = static_cast<float>(vector[4]);
        output.array[1][2] = static_cast<float>(vector[5]);
        output.array[2][0] = static_cast<float>(vector[6]);
        output.array[2][1] = static_cast<float>(vector[7]);
        output.array[2][2] = static_cast<float>(vector[8]);

        return output;
    }
} // fusion
