// Copyright 2026 Oh My SLAM contributors. Apache-2.0.
#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>

#include "angles/angles.h"
#include "dwb_plugins/standard_traj_generator.hpp"
#include "nav2_controller/plugins/pose_progress_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "omslam_operator_msgs/msg/operator_state.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace omslam_navigation
{

class SpeedAwareTrajectoryGenerator : public dwb_plugins::StandardTrajectoryGenerator
{
public:
  void initialize(const nav2_util::LifecycleNode::SharedPtr & node, const std::string & name) override
  {
    StandardTrajectoryGenerator::initialize(node, name);
    nominal_horizon_ = sim_time_;
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".minimum_projection_distance", rclcpp::ParameterValue(0.10));
    minimum_projection_ = node->get_parameter(name + ".minimum_projection_distance").as_double();
  }

  void startNewIteration(const nav_2d_msgs::msg::Twist2D & velocity) override
  {
    // Score across at least two local costmap cells even at the operator's
    // minimum wheel speed. Use the measured speed during braking so the
    // horizon does not grow abruptly while still moving faster than the cap.
    auto limits = kinematics_handler_->getKinematics();
    const auto speed = std::max(limits.getMaxSpeedXY(), std::hypot(velocity.x, velocity.y));
    // Two 5 cm cells take 14.05 s at the supported 1 wheel RPM.
    // Guard against an arbitrarily small positive external speed limit.
    sim_time_ = speed > 0.0 ? std::clamp(minimum_projection_ / speed, nominal_horizon_, std::max(nominal_horizon_, 15.0)) :
      nominal_horizon_;
    StandardTrajectoryGenerator::startNewIteration(velocity);
  }

private:
  double nominal_horizon_{0.0};
  double minimum_projection_{0.0};
};

class SpeedAwareProgressChecker : public nav2_controller::PoseProgressChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name) override
  {
    auto node = parent.lock();
    if (!node) {
      throw std::runtime_error("SpeedAwareProgressChecker '" + name + "' requires a live parent node");
    }
    PoseProgressChecker::initialize(parent, name);
    using State = omslam_operator_msgs::msg::OperatorState;
    // The server republishes its accepted limits at 10 Hz and retains the
    // most recent one for a controller that was configured or restarted later.
    subscription_ = node->create_subscription<State>(
      "/operator/state", rclcpp::QoS(1).transient_local(),
      [this](State::ConstSharedPtr state) {
        const double fraction = state->speed_percent / 100.0;
        const double linear = state->max_linear_velocity * fraction;
        const double angular = state->max_angular_velocity * fraction;
        if (state->operator_enabled && std::isfinite(fraction) && fraction > 0.0 && fraction <= 1.0 &&
          std::isfinite(linear) && linear > 0.0 && std::isfinite(angular) && angular > 0.0)
        {
          std::lock_guard<std::mutex> lock(limits_mutex_);
          linear_cap_ = linear;
          angular_cap_ = angular;
        } else {
          std::lock_guard<std::mutex> lock(limits_mutex_);
          linear_cap_ = angular_cap_ = std::numeric_limits<double>::infinity();
        }
      });
  }

  bool check(geometry_msgs::msg::PoseStamped & current) override
  {
    const auto pose = nav_2d_utils::poseToPose2D(current.pose);
    // Keep the configured stall deadline at every speed. Allow half of the
    // capped travel within it for acceleration and local trajectory selection.
    std::lock_guard<std::mutex> lock(limits_mutex_);
    const double interval = 0.5 * time_allowance_.seconds();
    const double distance = std::min(radius_, linear_cap_ * interval);
    const double angle = std::min(required_movement_angle_, angular_cap_ * interval);
    if (!baseline_pose_set_ || clock_->now() < baseline_time_ ||
      pose_distance(pose, baseline_pose_) > distance)
    {
      resetBaselinePose(pose);
      previous_yaw_ = pose.theta;
      unwrapped_yaw_ = low_yaw_ = high_yaw_ = 0.;
      return true;
    }
    unwrapped_yaw_ += angles::shortest_angular_distance(previous_yaw_, pose.theta);
    previous_yaw_ = pose.theta;
    if (unwrapped_yaw_ < low_yaw_ - angle || unwrapped_yaw_ > high_yaw_ + angle) {
      low_yaw_ = std::min(low_yaw_, unwrapped_yaw_);
      high_yaw_ = std::max(high_yaw_, unwrapped_yaw_);
      baseline_time_ = clock_->now();
      return true;
    }
    return clock_->now() - baseline_time_ <= time_allowance_;
  }

private:
  double previous_yaw_{0.}, unwrapped_yaw_{0.}, low_yaw_{0.}, high_yaw_{0.};
  std::mutex limits_mutex_;
  double linear_cap_{std::numeric_limits<double>::infinity()};
  double angular_cap_{std::numeric_limits<double>::infinity()};
  rclcpp::Subscription<omslam_operator_msgs::msg::OperatorState>::SharedPtr subscription_;
};

}  // namespace omslam_navigation

PLUGINLIB_EXPORT_CLASS(omslam_navigation::SpeedAwareTrajectoryGenerator, dwb_core::TrajectoryGenerator)
PLUGINLIB_EXPORT_CLASS(omslam_navigation::SpeedAwareProgressChecker, nav2_core::ProgressChecker)
