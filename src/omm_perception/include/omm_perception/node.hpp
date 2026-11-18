#pragma once
#include <rclcpp/rclcpp.hpp>

namespace omm_perception {
std::shared_ptr<rclcpp::Node> makeObstacleMemory(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
}
