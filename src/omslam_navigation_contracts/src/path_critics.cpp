// Copyright 2026 Oh My SLAM contributors. Apache-2.0.
#include <algorithm>
#include <cmath>
#include <vector>
#include "angles/angles.h"
#include "dwb_core/exceptions.hpp"
#include "dwb_critics/obstacle_footprint.hpp"
#include "dwb_critics/oscillation.hpp"
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

class PlannedOscillationCritic : public dwb_critics::OscillationCritic
{
public:
  bool prepare(const Pose & pose, const Twist & vel, const Pose & goal, const Path & plan) override
  {
    remaining_ = remainingTurn(pose, plan, costmap_ros_->getCostmap()->getResolution());
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
  double remaining_{0.};
};

class PlannedTurnCritic : public dwb_core::TrajectoryCritic
{
public:
  bool prepare(const Pose & pose, const Twist & velocity, const Pose &, const Path & path) override
  {
    translation_limit_ = std::abs(velocity.x);
    remaining_ = remainingTurn(pose, path, costmap_ros_->getCostmap()->getResolution());
    return true;
  }

  double scoreTrajectory(const Trajectory & trajectory) override
  {
    if (remaining_ == 0.) {return 0.;}
    if (trajectory.velocity.theta * remaining_ <= 0. ||
      std::abs(trajectory.velocity.x) > translation_limit_ + 1e-6) {
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
};
}  // namespace omslam_navigation
PLUGINLIB_EXPORT_CLASS(omslam_navigation::FilledFootprintCritic, dwb_core::TrajectoryCritic)
PLUGINLIB_EXPORT_CLASS(omslam_navigation::PlannedTurnCritic, dwb_core::TrajectoryCritic)
PLUGINLIB_EXPORT_CLASS(omslam_navigation::PlannedOscillationCritic, dwb_core::TrajectoryCritic)
