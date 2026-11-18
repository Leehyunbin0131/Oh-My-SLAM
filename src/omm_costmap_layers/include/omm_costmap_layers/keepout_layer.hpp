#pragma once
#include <mutex>
#include <nav2_costmap_2d/layer.hpp>
#include <omslam_operator_msgs/msg/keepout_zones.hpp>
#include <tf2/LinearMath/Vector3.hpp>

namespace omm_costmap_layers {
class KeepoutLayer : public nav2_costmap_2d::Layer {
public:
  void onInitialize() override;
  void reset() override {}
  bool isClearable() override { return false; }
  void updateBounds(double, double, double, double *, double *, double *, double *) override;
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override;
private:
  using Polygon = std::vector<tf2::Vector3>;
  std::mutex mutex_;
  omslam_operator_msgs::msg::KeepoutZones::ConstSharedPtr snapshot_;
  rclcpp::Subscription<omslam_operator_msgs::msg::KeepoutZones>::SharedPtr sub_;
  std::vector<Polygon> projected_;
};
} // namespace omm_costmap_layers
