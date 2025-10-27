//
// Created by antique on 10/12/25.
//

#ifndef FUSION_ROS2_FUSION_FILTER_H
#define FUSION_ROS2_FUSION_FILTER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "fusion_filter_ros2/fusion_filter_parameters.hpp"

#include "Fusion.h"

#define FUSION_ARRAY_SIZE 3

namespace fusion_filter
{
    class FusionFilterNode : public rclcpp::Node
    {
    public:
        explicit FusionFilterNode();
        ~FusionFilterNode() override = default;

        void init();

    private:
        void imu_callback(const sensor_msgs::msg::Imu& msg);
        void mag_callback(const sensor_msgs::msg::MagneticField& mag);

        std::array<double, 9> vector_to_covariance(const std::vector<double>& vec);
        FusionVector vector_to_fusion_vector(const std::vector<double>& vec);
        FusionMatrix vector_to_fusion_matrix(const std::vector<double>& vec);

    protected:
        std::shared_ptr<ParamListener> _param_listener;
        Params _params;

        rclcpp::Clock _steady_clock;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_filtered_pub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
        rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr _mag_sub;

        FusionVector _mag;
        rclcpp::Time _prev_time;

        // AHRS algorithms
        FusionOffset _offset;
        FusionAhrs _ahrs;
        FusionAhrsSettings _settings;

        // IMU calibration parameters
        FusionMatrix _gyroscope_misalignment;
        FusionVector _gyroscope_sensitivity;
        FusionVector _gyroscope_offset;
        FusionMatrix _accelerometer_misalignment;
        FusionVector _accelerometer_sensitivity;
        FusionVector _accelerometer_offset;
        FusionMatrix _soft_iron_matrix;
        FusionVector _hard_iron_offset;

        // filtered IMU data covariances
        std::array<double, 9> _orientation_covariance;
        std::array<double, 9> _angular_velocity_covariance;
        std::array<double, 9> _linear_acceleration_covariance;
    };
} // fusion

#endif //FUSION_ROS2_FUSION_FILTER_H
