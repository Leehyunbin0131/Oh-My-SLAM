#pragma once
#include <mutex>
#include <nav2_costmap_2d/layer.hpp>
#include <omm_perception_msgs/msg/obstacle_memory.hpp>
#include <tf2/LinearMath/Vector3.hpp>

namespace omm_costmap_layers {
class ObstacleMemoryLayer : public nav2_costmap_2d::Layer {
public:
  void onInitialize() override;
  // Ordinary Nav2 clear/reset never supplies free-space observations.
  void reset() override {}
  bool isClearable() override { return false; }
  void updateBounds(double, double, double, double *, double *, double *, double *) override;
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override;

private:
  void prepareProjection();
  double padding_ = .06, timeout_ = .30, radius_ = .085, archive_radius_ = .15;
  double last_now_ = -1.;
  std::string memory_frame_ = "odom";
  std::mutex mutex_;
  bool new_epoch_ = false;
  std::vector<tf2::Vector3> transformed_, archived_;
  omm_perception_msgs::msg::ObstacleMemory::ConstSharedPtr snapshot_, accepted_;
  rclcpp::Subscription<omm_perception_msgs::msg::ObstacleMemory>::SharedPtr sub_;
  rclcpp::JumpHandler::SharedPtr jump_handler_;
};
} // namespace omm_costmap_layers
