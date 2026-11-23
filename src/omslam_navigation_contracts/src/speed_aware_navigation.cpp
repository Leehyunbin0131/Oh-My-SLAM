// Copyright 2026 Oh My SLAM contributors. Apache-2.0.
#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "dwb_plugins/standard_traj_generator.hpp"
#include "nav2_controller/plugins/pose_progress_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "omslam_operator_msgs/msg/operator_state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "drive_geometry.hpp"

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
    geometry_ = DriveGeometry::read(node, name);
    auto limits = kinematics_handler_->getKinematics();
    base_linear_ = limits.getMaxSpeedXY();
    base_angular_ = limits.getMaxTheta();
    rpm_cap_ = geometry_.max_rpm;
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    if (!std::isfinite(speed_limit) || speed_limit < 0.) {return;}
    // Nav2 uses zero to clear a speed limit. Absolute limits use the same
    // fraction of the base linear speed as the stock kinematics handler.
    const double fraction = speed_limit == 0. ? 1. :
      std::min(1., speed_limit / (percentage ? 100. : base_linear_));
    requested_fraction_.store(fraction);
  }

  void startNewIteration(const nav_2d_msgs::msg::Twist2D & velocity) override
  {
    // The speed-limit callback can run during scoring. Apply both caps on
    // the controller thread at the start of an iteration.
    const double fraction = requested_fraction_.load();
    if (fraction != active_fraction_) {
      StandardTrajectoryGenerator::setSpeedLimit(fraction * 100., true);
      rpm_cap_ = geometry_.max_rpm * fraction;
      if (geometry_.atFloor(rpm_cap_)) {rpm_cap_ = geometry_.min_rpm;}
      active_fraction_ = fraction;
    }
    // Score across at least two local costmap cells even at the operator's
    // minimum wheel speed. Use the measured speed during braking so the
    // horizon does not grow abruptly while still moving faster than the cap.
    auto limits = kinematics_handler_->getKinematics();
    const auto speed = std::max(limits.getMaxSpeedXY(), std::hypot(velocity.x, velocity.y));
    sim_time_ = speed > 0.0 ? std::clamp(minimum_projection_ / speed, nominal_horizon_, std::max(nominal_horizon_, 15.0)) :
      nominal_horizon_;
    StandardTrajectoryGenerator::startNewIteration(velocity);
    current_velocity_ = velocity;
    candidates_.clear();
    next_ = 0;
    while (StandardTrajectoryGenerator::hasMoreTwists()) {
      addCandidate(StandardTrajectoryGenerator::nextTwist());
    }
    // Explicit axis samples keep the executable boundary in the search grid
    // even when percentage scaling would put the entire rotation grid below it.
    nav_2d_msgs::msg::Twist2D axis;
    const double linear = geometry_.straightFloor();
    if (linear <= base_linear_) {
      axis.x = linear;
      addCandidate(axis);
    }
    if (geometry_.rotationFloor() <= base_angular_) {
      axis.x = 0.;
      axis.theta = geometry_.rotationFloor();
      addCandidate(axis);
      axis.theta = -axis.theta;
      addCandidate(axis);
    }
    candidates_.emplace_back();
  }

  bool hasMoreTwists() override {return next_ < candidates_.size();}
  nav_2d_msgs::msg::Twist2D nextTwist() override {return candidates_[next_++];}

protected:
  std::vector<double> getTimeSteps(const nav_2d_msgs::msg::Twist2D & cmd_vel) override
  {
    if (cmd_vel.x == 0. && cmd_vel.theta == 0.) {
      // Preserve spatial resolution along the braking trajectory.
      return StandardTrajectoryGenerator::getTimeSteps(current_velocity_);
    }
    return StandardTrajectoryGenerator::getTimeSteps(cmd_vel);
  }

  geometry_msgs::msg::Pose2D computeNewPosition(
    const geometry_msgs::msg::Pose2D pose, const nav_2d_msgs::msg::Twist2D & velocity,
    double dt) override
  {
    // Keep the smoother-side ramp in the integrator, but move only with the
    // twist the wheels can apply at each step.
    return StandardTrajectoryGenerator::computeNewPosition(
      pose, geometry_.applied(velocity, rpm_cap_), dt);
  }

private:
  void addCandidate(const nav_2d_msgs::msg::Twist2D & candidate)
  {
    const auto applied = geometry_.applied(candidate, rpm_cap_);
    if (applied.x != 0. || applied.theta != 0.) {
      candidates_.push_back(applied);
    }
  }

  DriveGeometry geometry_{};
  std::atomic<double> requested_fraction_{1.};
  double active_fraction_{1.};
  double rpm_cap_{0.}, base_linear_{0.}, base_angular_{0.};
  nav_2d_msgs::msg::Twist2D current_velocity_;
  std::vector<nav_2d_msgs::msg::Twist2D> candidates_;
  size_t next_{0};
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
        const double linear = state->auto_linear_limit;
        const double angular = state->auto_angular_limit;
        if (state->operator_enabled &&
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
