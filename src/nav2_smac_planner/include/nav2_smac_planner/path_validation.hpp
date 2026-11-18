// Copyright 2026 Oh_My_SLAM contributors
// Licensed under the Apache License, Version 2.0.
#ifndef NAV2_SMAC_PLANNER__PATH_VALIDATION_HPP_
#define NAV2_SMAC_PLANNER__PATH_VALIDATION_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

#include "angles/angles.h"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_smac_planner
{
// Final output uses continuous yaw, rather than the lattice's orientation bins.
// Validate the filled convex footprint, including cells wholly inside it, and
// interpolate so no footprint vertex travels more than half a cell per sample.
// Caller holds the costmap mutex throughout planning, smoothing and validation.
inline bool isPathCollisionFree(
  const nav_msgs::msg::Path & path, nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint, bool traverse_unknown)
{
  if (path.poses.empty() || footprint.size() < 3) {
    return false;
  }
  double radius = 0.0;
  for (const auto & p : footprint) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {return false;}
    radius = std::max(radius, std::hypot(p.x, p.y));
  }
  auto valid_pose = [&](double x, double y, double yaw) {
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {return false;}
      unsigned int mx, my;
      if (!costmap.worldToMap(x, y, mx, my)) {return false;}
      const auto center = costmap.getCost(mx, my);
      if ((center == nav2_costmap_2d::NO_INFORMATION && !traverse_unknown) ||
        center == nav2_costmap_2d::LETHAL_OBSTACLE ||
        center == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        return false;
      }
      std::vector<nav2_costmap_2d::MapLocation> polygon;
      polygon.reserve(footprint.size());
      const double c = std::cos(yaw), s = std::sin(yaw);
      for (const auto & p : footprint) {
        nav2_costmap_2d::MapLocation cell;
        if (!costmap.worldToMap(x + p.x * c - p.y * s, y + p.x * s + p.y * c,
            cell.x, cell.y))
        {
          return false;
        }
        polygon.push_back(cell);
      }
      std::vector<nav2_costmap_2d::MapLocation> cells;
      costmap.convexFillCells(polygon, cells);
      for (const auto & cell : cells) {
        const auto cost = costmap.getCost(cell.x, cell.y);
        if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
          (cost == nav2_costmap_2d::NO_INFORMATION && !traverse_unknown))
        {
          return false;
        }
      }
      return true;
    };
  double x = path.poses.front().pose.position.x;
  double y = path.poses.front().pose.position.y;
  double yaw = tf2::getYaw(path.poses.front().pose.orientation);
  if (!valid_pose(x, y, yaw)) {return false;}
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & pose = path.poses[i].pose;
    const double nx = pose.position.x, ny = pose.position.y;
    const double next_yaw = tf2::getYaw(pose.orientation);
    if (!valid_pose(nx, ny, next_yaw)) {return false;}
    const double dyaw = angles::shortest_angular_distance(yaw, next_yaw);
    const double travel = std::hypot(nx - x, ny - y) + radius * std::abs(dyaw);
    const auto samples = static_cast<size_t>(std::max(1.0,
        std::ceil(travel / (0.5 * costmap.getResolution()))));
    for (size_t j = 1; j <= samples; ++j) {
      const double t = static_cast<double>(j) / samples;
      if (!valid_pose(x + t * (nx - x), y + t * (ny - y), yaw + t * dyaw)) {
        return false;
      }
    }
    x = nx; y = ny; yaw = next_yaw;
  }
  return true;
}

// A failed smoother can still modify its path. Validate regardless of its
// return value, and never assume the raw path is safe merely because A* found it.
inline bool validateOrRestorePath(
  nav_msgs::msg::Path & candidate, const nav_msgs::msg::Path & raw,
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint, bool traverse_unknown)
{
  if (isPathCollisionFree(candidate, costmap, footprint, traverse_unknown)) {return true;}
  if (!isPathCollisionFree(raw, costmap, footprint, traverse_unknown)) {
    throw nav2_core::NoValidPathCouldBeFound("Final and original paths fail footprint validation");
  }
  candidate = raw;
  return false;
}
}  // namespace nav2_smac_planner
#endif  // NAV2_SMAC_PLANNER__PATH_VALIDATION_HPP_
