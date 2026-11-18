#include "omm_perception/node.hpp"
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(omm_perception::makeObstacleMemory());
  rclcpp::shutdown();
}
