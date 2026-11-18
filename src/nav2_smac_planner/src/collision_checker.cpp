// Copyright (c) 2021, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/path_validation.hpp"
#include "nav2_smac_planner/utils.hpp"

namespace nav2_smac_planner
{

GridCollisionChecker::GridCollisionChecker(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  unsigned int num_quantizations,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
: FootprintCollisionChecker(costmap_ros ? costmap_ros->getCostmap() : nullptr)
{
  if (node) {
    clock_ = node->get_clock();
    logger_ = node->get_logger();
  }

  if (costmap_ros) {
    costmap_ros_ = costmap_ros;
  }

  // Convert number of regular bins into angles
  float bin_size = 2 * M_PI / static_cast<float>(num_quantizations);
  angles_.reserve(num_quantizations);
  for (unsigned int i = 0; i != num_quantizations; i++) {
    angles_.push_back(bin_size * i);
  }
}

// GridCollisionChecker::GridCollisionChecker(
//   nav2_costmap_2d::Costmap2D * costmap,
//   std::vector<float> & angles)
// : FootprintCollisionChecker(costmap),
//   angles_(angles)
// {
// }

void GridCollisionChecker::setFootprint(
  const nav2_costmap_2d::Footprint & footprint,
  const bool & radius,
  const double & possible_collision_cost)
{
  possible_collision_cost_ = static_cast<float>(possible_collision_cost);
  if (possible_collision_cost_ <= 0.0f) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 1000,
      "Inflation layer either not found or inflation is not set sufficiently for "
      "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
      " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
      "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
      " for full instructions. This will substantially impact run-time performance.");
  }

  footprint_is_radius_ = radius;

  // Use radius, no caching required
  if (radius) {
    return;
  }

  // No change, no updates required
  if (footprint == unoriented_footprint_) {
    return;
  }

  oriented_footprints_.clear();
  oriented_footprints_.reserve(angles_.size());
  double sin_th, cos_th;
  geometry_msgs::msg::Point new_pt;
  const unsigned int footprint_size = footprint.size();

  // Precompute the orientation bins for checking to use
  for (unsigned int i = 0; i != angles_.size(); i++) {
    sin_th = sin(angles_[i]);
    cos_th = cos(angles_[i]);
    nav2_costmap_2d::Footprint oriented_footprint;
    oriented_footprint.reserve(footprint_size);

    for (unsigned int j = 0; j < footprint_size; j++) {
      new_pt.x = footprint[j].x * cos_th - footprint[j].y * sin_th;
      new_pt.y = footprint[j].x * sin_th + footprint[j].y * cos_th;
      oriented_footprint.push_back(new_pt);
    }

    oriented_footprints_.push_back(oriented_footprint);
  }

  unoriented_footprint_ = footprint;
}

void GridCollisionChecker::prepareLatticeFootprint(bool traverse_unknown)
{
  obstacle_sums_.clear();
  if (footprint_is_radius_) {return;}
  lattice_traverse_unknown_ = traverse_unknown;
  footprint_radius_cells_ = 0.0;
  for (const auto & p : unoriented_footprint_) {
    footprint_radius_cells_ = std::max(
      footprint_radius_cells_, std::hypot(p.x, p.y) / costmap_->getResolution());
  }
  const size_t width = costmap_->getSizeInCellsX(), height = costmap_->getSizeInCellsY();
  const size_t stride = width + 1;
  obstacle_sums_.resize(stride * (height + 1), 0);
  const auto * costs = costmap_->getCharMap();
  for (size_t y = 0; y < height; ++y) {
    uint32_t row = 0;
    for (size_t x = 0; x < width; ++x) {
      const auto cell = costs[y * width + x];
      if (cell == OCCUPIED || (cell == UNKNOWN && !traverse_unknown)) {++row;}
      obstacle_sums_[(y + 1) * stride + x + 1] = obstacle_sums_[y * stride + x + 1] + row;
    }
  }
}

bool GridCollisionChecker::containsBlockedCells(
  unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1) const
{
  const size_t stride = static_cast<size_t>(costmap_->getSizeInCellsX()) + 1;
  const size_t right = static_cast<size_t>(x1) + 1;
  const size_t upper = static_cast<size_t>(y0) * stride, lower = (static_cast<size_t>(y1) + 1) * stride;
  return obstacle_sums_[lower + right] - obstacle_sums_[upper + right] !=
         obstacle_sums_[lower + x0] - obstacle_sums_[upper + x0];
}

bool GridCollisionChecker::bodyCollides(double wx, double wy, double yaw)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  unsigned int min_x = costmap_->getSizeInCellsX(), min_y = costmap_->getSizeInCellsY();
  unsigned int max_x = 0, max_y = 0;
  polygon_.clear();
  for (const auto & p : unoriented_footprint_) {
    nav2_costmap_2d::MapLocation cell;
    if (!costmap_->worldToMap(wx + p.x * c - p.y * s, wy + p.x * s + p.y * c, cell.x, cell.y)) {
      return true;
    }
    min_x = std::min(min_x, cell.x); max_x = std::max(max_x, cell.x);
    min_y = std::min(min_y, cell.y); max_y = std::max(max_y, cell.y);
    polygon_.push_back(cell);
  }
  if (polygon_.size() < 3) {return true;}
  if (!containsBlockedCells(min_x, min_y, max_x, max_y)) {return false;}

  // Costmap2D::convexFillCells fills between outline extremes in each column.
  // Use the same rasterized outline and count blocked cells across each whole
  // span, without sorting and materializing the filled polygon for each sample.
  outline_.clear();
  costmap_->polygonOutlineCells(polygon_, outline_);
  columns_.assign(max_x - min_x + 1, {costmap_->getSizeInCellsY(), 0u});
  for (const auto & cell : outline_) {
    auto & span = columns_[cell.x - min_x];
    span.first = std::min(span.first, cell.y);
    span.second = std::max(span.second, cell.y);
  }
  for (unsigned int x = min_x; x <= max_x; ++x) {
    const auto & span = columns_[x - min_x];
    if (span.first <= span.second && containsBlockedCells(x, span.first, x, span.second)) {return true;}
  }
  return false;
}

bool GridCollisionChecker::latticeSweepCollides(
  float x, float y, double yaw, float nx, float ny, double next_yaw)
{
  if (obstacle_sums_.empty()) {return false;}
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
    !std::isfinite(nx) || !std::isfinite(ny) || !std::isfinite(next_yaw)) {return true;}
  // One extra cell covers roundoff in continuous grid-to-world conversion.
  const double r = footprint_radius_cells_ + 1.0;
  const double x0 = std::floor(std::min(x, nx) - r), x1 = std::floor(std::max(x, nx) + r);
  const double y0 = std::floor(std::min(y, ny) - r), y1 = std::floor(std::max(y, ny) + r);
  const bool clear_body = x0 >= 0 && y0 >= 0 && x1 < costmap_->getSizeInCellsX() &&
    y1 < costmap_->getSizeInCellsY() && !containsBlockedCells(x0, y0, x1, y1);
  const auto start = getWorldCoords(x, y, costmap_).position;
  const auto end = getWorldCoords(nx, ny, costmap_).position;
  const double dx = end.x - start.x, dy = end.y - start.y;
  const double dyaw = angles::shortest_angular_distance(yaw, next_yaw);
  const double travel_cells = std::hypot(dx, dy) / costmap_->getResolution() +
    footprint_radius_cells_ * std::abs(dyaw);
  const size_t samples = static_cast<size_t>(std::max(1.0, std::ceil(2.0 * travel_cells)));
  for (size_t i = 0; i <= samples; ++i) {
    const double t = static_cast<double>(i) / samples;
    const double wx = start.x + t * dx, wy = start.y + t * dy;
    unsigned int mx, my;
    if (!costmap_->worldToMap(wx, wy, mx, my)) {return true;}
    const auto center = costmap_->getCost(mx, my);
    if (center == INSCRIBED || center == OCCUPIED || (center == UNKNOWN && !lattice_traverse_unknown_)) {
      return true;
    }
    if (!clear_body && bodyCollides(wx, wy, yaw + t * dyaw)) {return true;}
  }
  return false;
}

bool GridCollisionChecker::inCollision(
  const float & x,
  const float & y,
  const float & angle_bin,
  const bool & traverse_unknown)
{
  // Check to make sure cell is inside the map
  if (outsideRange(costmap_->getSizeInCellsX(), x) ||
    outsideRange(costmap_->getSizeInCellsY(), y))
  {
    return true;
  }

  // Continuous map coordinates use cell boundaries, as worldToMapContinuous does.
  // Assumes setFootprint already set.
  center_cost_ = static_cast<float>(costmap_->getCost(
    static_cast<unsigned int>(x), static_cast<unsigned int>(y)));

  if (!footprint_is_radius_) {
    // if footprint, then we check for the footprint's points, but first see
    // if the robot is even potentially in an inscribed collision
    if (center_cost_ < possible_collision_cost_ && possible_collision_cost_ > 0.0f) {
      return false;
    }

    // If its inscribed, in collision, or unknown in the middle,
    // no need to even check the footprint, its invalid
    if (center_cost_ == UNKNOWN && !traverse_unknown) {
      return true;
    }

    if (center_cost_ == INSCRIBED || center_cost_ == OCCUPIED) {
      return true;
    }

    // if possible inscribed, need to check actual footprint pose.
    // Use precomputed oriented footprints are done on initialization,
    // offset by translation value to collision check
    // Use the same continuous conversion as the returned path. mapToWorld takes
    // integer cell indices and would snap this pose to a cell center.
    const auto world_pose = getWorldCoords(x, y, costmap_);
    const double wx = world_pose.position.x;
    const double wy = world_pose.position.y;
    geometry_msgs::msg::Point new_pt;
    const nav2_costmap_2d::Footprint & oriented_footprint = oriented_footprints_[angle_bin];
    nav2_costmap_2d::Footprint current_footprint;
    current_footprint.reserve(oriented_footprint.size());
    for (unsigned int i = 0; i < oriented_footprint.size(); ++i) {
      new_pt.x = wx + oriented_footprint[i].x;
      new_pt.y = wy + oriented_footprint[i].y;
      current_footprint.push_back(new_pt);
    }

    float footprint_cost = static_cast<float>(footprintCost(current_footprint));

    if (footprint_cost == UNKNOWN && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return footprint_cost >= OCCUPIED;
  } else {
    // if radius, then we can check the center of the cost assuming inflation is used
    if (center_cost_ == UNKNOWN && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return center_cost_ >= INSCRIBED;
  }
}

bool GridCollisionChecker::inCollision(
  const unsigned int & i,
  const bool & traverse_unknown)
{
  if (i >= costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY()) {
    return true;
  }
  center_cost_ = costmap_->getCost(i);
  if (center_cost_ == UNKNOWN && traverse_unknown) {
    return false;
  }

  // if occupied or unknown and not to traverse unknown space
  return center_cost_ >= INSCRIBED;
}

float GridCollisionChecker::getCost()
{
  // Assumes inCollision called prior
  return static_cast<float>(center_cost_);
}

bool GridCollisionChecker::outsideRange(const unsigned int & max, const float & value)
{
  return !std::isfinite(value) || value < 0.0f || value >= max;
}

}  // namespace nav2_smac_planner
