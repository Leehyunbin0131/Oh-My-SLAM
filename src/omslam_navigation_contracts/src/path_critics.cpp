// Copyright 2026 Oh My SLAM contributors. Apache-2.0.
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include "angles/angles.h"
#include "dwb_core/exceptions.hpp"
#include "dwb_critics/obstacle_footprint.hpp"
#include "dwb_critics/oscillation.hpp"
#include "dwb_critics/rotate_to_goal.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace omslam_navigation
{
using Pose = geometry_msgs::msg::Pose2D;
using Twist = nav_2d_msgs::msg::Twist2D;
using Path = nav_2d_msgs::msg::Path2D;
using Trajectory = dwb_msgs::msg::Trajectory2D;

// Preserve DWB's obstacle clearance score. Check the filled footprint throughout
// the sweep as well. A summed-area table skips rasterization in clear regions.
class FilledFootprintCritic : public dwb_critics::ObstacleFootprintCritic
{
public:
  bool prepare(const Pose & pose, const Twist & vel, const Pose & goal, const Path & plan) override
  {
    if (!ObstacleFootprintCritic::prepare(pose, vel, goal, plan)) {return false;}
    start_ = pose;
    radius_ = 0.;
    for (const auto & p : footprint_spec_) {radius_ = std::max(radius_, std::hypot(p.x, p.y));}
    stride_ = costmap_->getSizeInCellsX() + 1;
    occupied_.assign(stride_ * (costmap_->getSizeInCellsY() + 1), 0);
    for (unsigned y = 0; y < costmap_->getSizeInCellsY(); ++y) {
      unsigned row = 0;
      for (unsigned x = 0; x < costmap_->getSizeInCellsX(); ++x) {
        row += costmap_->getCost(x, y) >= nav2_costmap_2d::LETHAL_OBSTACLE;
        occupied_[(y + 1) * stride_ + x + 1] = row + occupied_[y * stride_ + x + 1];
      }
    }
    return true;
  }

  double scoreTrajectory(const Trajectory & trajectory) override
  {
    Pose from = start_;
    check(from);
    for (const auto & to : trajectory.poses) {
      const double turn = angles::shortest_angular_distance(from.theta, to.theta);
      const double travel = std::hypot(to.x - from.x, to.y - from.y) + radius_ * std::abs(turn);
      const int steps = std::max(1, static_cast<int>(std::ceil(travel / (0.5 * costmap_->getResolution()))));
      for (int i = 1; i <= steps; ++i) {
        const double f = static_cast<double>(i) / steps;
        Pose p;
        p.x = from.x + (to.x - from.x) * f;
        p.y = from.y + (to.y - from.y) * f;
        p.theta = from.theta + turn * f;
        check(p);
      }
      from = to;
    }
    return ObstacleFootprintCritic::scoreTrajectory(trajectory);
  }

private:
  void check(const Pose & p)
  {
    unsigned x0, y0, x1, y1;
    if (costmap_->worldToMap(p.x - radius_, p.y - radius_, x0, y0) &&
      costmap_->worldToMap(p.x + radius_, p.y + radius_, x1, y1) &&
      occupied_[(y1 + 1) * stride_ + x1 + 1] + occupied_[y0 * stride_ + x0] ==
      occupied_[y0 * stride_ + x1 + 1] + occupied_[(y1 + 1) * stride_ + x0]) {return;}
    std::vector<nav2_costmap_2d::MapLocation> polygon, cells;
    for (const auto & vertex : dwb_critics::getOrientedFootprint(p, footprint_spec_)) {
      nav2_costmap_2d::MapLocation cell;
      if (!costmap_->worldToMap(vertex.x, vertex.y, cell.x, cell.y)) {
        throw dwb_core::IllegalTrajectoryException(name_, "Footprint outside local costmap");
      }
      polygon.push_back(cell);
    }
    costmap_->convexFillCells(polygon, cells);
    for (const auto & cell : cells) {
      if (costmap_->getCost(cell.x, cell.y) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        throw dwb_core::IllegalTrajectoryException(name_, "Occupied swept footprint");
      }
    }
  }
  Pose start_;
  double radius_{0.};
  size_t stride_{0};
  std::vector<unsigned> occupied_;
};

static double remainingTurn(const Pose & pose, const Path & path, double resolution)
{
  if (path.poses.size() < 2) {return 0.;}
  const auto & anchor = path.poses.front();
  if (std::hypot(pose.x - anchor.x, pose.y - anchor.y) > resolution) {
    return 0.;
  }
  double sweep = 0.;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & next = path.poses[i];
    if (std::hypot(next.x - anchor.x, next.y - anchor.y) > 0.001) {break;}
    sweep += angles::shortest_angular_distance(path.poses[i - 1].theta, next.theta);
  }
  if (std::abs(sweep) < 0.075) {return 0.;}
  double progress = angles::shortest_angular_distance(anchor.theta, pose.theta);
  if (std::abs(sweep) > M_PI && progress * sweep < 0. &&
    std::abs(progress) > 2 * M_PI - std::abs(sweep) - 0.075) {
    progress += std::copysign(2 * M_PI, sweep);
  }
  const double remaining = sweep - progress;
  return remaining * std::copysign(1., sweep) > 0.075 ? remaining : 0.;
}

class CurrentPoseRotateToGoalCritic : public dwb_critics::RotateToGoalCritic
{
public:
  void onInit() override
  {
    RotateToGoalCritic::onInit();
    xy_goal_tolerance_ = node_.lock()->get_parameter(dwb_plugin_name_ + ".xy_goal_tolerance").as_double();
  }

  bool prepare(const Pose & pose, const Twist & vel, const Pose & goal, const Path & plan) override
  {
    if (std::hypot(goal.x - pose.x, goal.y - pose.y) > xy_goal_tolerance_) {
      // The stopped goal checker also rechecks current XY after braking drift.
      reset();
    }
    return RotateToGoalCritic::prepare(pose, vel, goal, plan);
  }

private:
  double xy_goal_tolerance_{0.};
};

class PlannedOscillationCritic : public dwb_critics::OscillationCritic
{
public:
  void onInit() override
  {
    OscillationCritic::onInit();
    xy_goal_tolerance_ = node_.lock()->get_parameter(dwb_plugin_name_ + ".xy_goal_tolerance").as_double();
  }

  bool prepare(const Pose & pose, const Twist & vel, const Pose & goal, const Path & plan) override
  {
    remaining_ = remainingTurn(pose, plan, costmap_ros_->getCostmap()->getResolution());
    if (remaining_ == 0. && std::hypot(goal.x - pose.x, goal.y - pose.y) <= xy_goal_tolerance_) {
      remaining_ = angles::shortest_angular_distance(pose.theta, goal.theta);
    }
    return OscillationCritic::prepare(pose, vel, goal, plan);
  }

  double scoreTrajectory(const Trajectory & trajectory) override
  {
    const auto & v = trajectory.velocity;
    if (v.x == 0. && v.y == 0. && v.theta * remaining_ > 0.) {
      // A previously selected contrary spin must not lock out the only planned
      // turn. The footprint and all other trajectory critics still apply.
      return 0.;
    }
    return OscillationCritic::scoreTrajectory(trajectory);
  }
private:
  double remaining_{0.}, xy_goal_tolerance_{0.};
};

class PlannedTurnCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override
  {
    auto node = node_.lock();
    const auto number = [&](const std::string & name, double fallback) {
        const auto key = dwb_plugin_name_ + "." + name;
        nav2_util::declare_parameter_if_not_declared(node, key, rclcpp::ParameterValue(fallback));
        return node->get_parameter(key).as_double();
      };
    xy_goal_tolerance_ = number("xy_goal_tolerance", 0.05);
    yaw_goal_tolerance_ = number("yaw_goal_tolerance", 0.15);
  }

  bool prepare(const Pose & pose, const Twist & velocity, const Pose & goal, const Path & path) override
  {
    translation_limit_ = std::abs(velocity.x);
    remaining_ = remainingTurn(pose, path, costmap_ros_->getCostmap()->getResolution());
    const double yaw_error = angles::shortest_angular_distance(pose.theta, goal.theta);
    const bool xy_reached = std::hypot(goal.x - pose.x, goal.y - pose.y) <= xy_goal_tolerance_;
    pose_reached_ = xy_reached && std::abs(yaw_error) <= yaw_goal_tolerance_;
    if (xy_reached && remaining_ == 0.) {remaining_ = yaw_error;}
    return true;
  }

  double scoreTrajectory(const Trajectory & trajectory) override
  {
    const auto & v = trajectory.velocity;
    const bool stopping = v.x == 0. && v.y == 0. && v.theta == 0.;
    if (stopping) {
      // Let a collision-checked stop brake at the goal or serve as a fallback
      // when all moving candidates are illegal. Never let it win a motion tie.
      return pose_reached_ ? 0. : std::numeric_limits<float>::max();
    }
    if (pose_reached_) {
      throw dwb_core::IllegalTrajectoryException(name_, "Brake within final pose tolerance");
    }
    if (remaining_ == 0.) {return 0.;}
    if (v.theta * remaining_ <= 0. || std::abs(v.x) > translation_limit_ + 1e-6) {
      throw dwb_core::IllegalTrajectoryException(name_, "Rotation opposes planned in-place turn");
    }
    double motion = 0.;
    for (size_t i = 1; i < trajectory.poses.size(); ++i) {
      motion += angles::shortest_angular_distance(trajectory.poses[i - 1].theta, trajectory.poses[i].theta);
    }
    return std::abs(remaining_ - motion);
  }
private:
  double remaining_{0.}, translation_limit_{0.};
  double xy_goal_tolerance_{0.}, yaw_goal_tolerance_{0.};
  bool pose_reached_{false};
};
}  // namespace omslam_navigation
PLUGINLIB_EXPORT_CLASS(omslam_navigation::FilledFootprintCritic, dwb_core::TrajectoryCritic)
PLUGINLIB_EXPORT_CLASS(omslam_navigation::PlannedTurnCritic, dwb_core::TrajectoryCritic)
PLUGINLIB_EXPORT_CLASS(omslam_navigation::PlannedOscillationCritic, dwb_core::TrajectoryCritic)
PLUGINLIB_EXPORT_CLASS(omslam_navigation::CurrentPoseRotateToGoalCritic, dwb_core::TrajectoryCritic)
