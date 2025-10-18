//
// Created by antique on 10/12/25.
//

#include <rclcpp/rclcpp.hpp>
#include "fusion_ros2/fusion_filter_node.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<fusion::FusionFilterNode> node = std::make_shared<fusion::FusionFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
