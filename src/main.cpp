//
// Created by antique on 10/12/25.
//

#include <rclcpp/rclcpp.hpp>
#include "fusion_filter_ros2/fusion_filter_node.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<fusion_filter::FusionFilterNode> node = std::make_shared<fusion_filter::FusionFilterNode>();
    node->init();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
