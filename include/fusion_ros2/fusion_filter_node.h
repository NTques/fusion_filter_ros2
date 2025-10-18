//
// Created by antique on 10/12/25.
//

#ifndef FUSION_ROS2_FUSION_FILTER_H
#define FUSION_ROS2_FUSION_FILTER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "Fusion.h"

#define FUSION_ARRAY_SIZE 3

namespace fusion
{
    class FusionFilterNode : public rclcpp::Node
    {
    public:
        explicit FusionFilterNode();
        ~FusionFilterNode() override = default;

    private:
        FusionVector get_calibration_parameter_as_fusion_vector(const std::string& name,
                                                                const std::vector<double>& default_value);
        FusionMatrix get_calibration_parameter_as_fusion_matrix(const std::string& name,
                                                                const std::vector<double>& default_value);

        void imu_callback(const sensor_msgs::msg::Imu& msg);
        void mag_callback(const sensor_msgs::msg::MagneticField& mag);

    protected:
        int _sample_rate;
        bool _use_magnetic_field;

        rclcpp::Clock _steady_clock;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_filtered_pub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
        rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr _mag_sub;

        FusionVector _mag;
        rclcpp::Time _prev_time;

        // imu calibration parameters
        FusionMatrix _gyroscope_misalignment;
        FusionVector _gyroscope_sensitivity;
        FusionVector _gyroscope_offset;
        FusionMatrix _accelerometer_misalignment;
        FusionVector _accelerometer_sensitivity;
        FusionVector _accelerometer_offset;
        FusionMatrix _soft_iron_matrix;
        FusionVector _hard_iron_offset;

        // AHRS algorithms
        FusionOffset _offset;
        FusionAhrs _ahrs;
        FusionAhrsSettings _settings;
    };
} // fusion

#endif //FUSION_ROS2_FUSION_FILTER_H
