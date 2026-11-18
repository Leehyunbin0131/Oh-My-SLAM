// Copyright 2026 Oh_My_SLAM contributors. Apache-2.0.
#include <limits>
#include "gtest/gtest.h"
#include "nav2_smac_planner/path_validation.hpp"
#include "nav2_smac_planner/smoother.hpp"
#include "nav2_smac_planner/collision_checker.hpp"

namespace
{
using nav2_smac_planner::isPathCollisionFree;
using nav2_smac_planner::validateOrRestorePath;

std::vector<geometry_msgs::msg::Point> footprint()
{
  std::vector<geometry_msgs::msg::Point> out;
  for (const auto xy : {std::pair<double, double>{.11, .28}, {.11, -.28},
      {-.41, -.28}, {-.41, .28}})
  {
    geometry_msgs::msg::Point p;
    p.x = xy.first; p.y = xy.second; out.push_back(p);
  }
  return out;
}
geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = x; p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2);
  p.pose.orientation.w = std::cos(yaw / 2);
  return p;
}
void obstacle(nav2_costmap_2d::Costmap2D & map, double x, double y, unsigned char cost = 254)
{
  unsigned int mx, my;
  ASSERT_TRUE(map.worldToMap(x, y, mx, my)); map.setCost(mx, my, cost);
}
}

TEST(FinalPath, RejectsRearInteriorUnknownRotationAndMapBoundary)
{
  nav2_costmap_2d::Costmap2D map(160, 160, .05, -4, -4, 0);
  nav_msgs::msg::Path path; path.poses = {pose(0, 0)};
  EXPECT_TRUE(isPathCollisionFree(path, map, footprint(), false));
  obstacle(map, -.30, .1);  // Entirely inside rear footprint, away from center/perimeter.
  EXPECT_FALSE(isPathCollisionFree(path, map, footprint(), false));
  obstacle(map, -.30, .1, 255);
  EXPECT_FALSE(isPathCollisionFree(path, map, footprint(), false));
  EXPECT_TRUE(isPathCollisionFree(path, map, footprint(), true));
  obstacle(map, -.30, .1, 0);
  obstacle(map, -.1, -.35);
  EXPECT_TRUE(isPathCollisionFree(path, map, footprint(), false));
  path.poses = {pose(0, 0, M_PI / 2)};
  EXPECT_FALSE(isPathCollisionFree(path, map, footprint(), false));
  path.poses = {pose(-3.8, 0)};  // Center in map; rear corners outside.
  EXPECT_FALSE(isPathCollisionFree(path, map, footprint(), false));
}

TEST(FinalPath, SamplesTranslationAndInPlaceRotation)
{
  nav2_costmap_2d::Costmap2D map(160, 160, .05, -4, -4, 0);
  nav_msgs::msg::Path path; path.poses = {pose(-1, 0), pose(1, 0)};
  obstacle(map, 0, 0);
  for (const auto & p : path.poses) {
    nav_msgs::msg::Path single; single.poses = {p};
    ASSERT_TRUE(isPathCollisionFree(single, map, footprint(), false));
  }
  EXPECT_FALSE(isPathCollisionFree(path, map, footprint(), false));
  obstacle(map, 0, 0, 0);
  obstacle(map, .33, -.33);
  path.poses = {pose(0, 0), pose(0, 0, M_PI)};
  for (const auto & p : path.poses) {
    nav_msgs::msg::Path single; single.poses = {p};
    ASSERT_TRUE(isPathCollisionFree(single, map, footprint(), false));
  }
  EXPECT_FALSE(isPathCollisionFree(path, map, footprint(), false));
}

TEST(FinalPath, FallsBackOnlyToValidatedOriginal)
{
  nav2_costmap_2d::Costmap2D map(160, 160, .05, -4, -4, 0);
  nav_msgs::msg::Path candidate, raw;
  candidate.poses = {pose(-1, 0), pose(1, 0)};
  raw.poses = {pose(-1, 0), pose(-1, 1), pose(1, 1), pose(1, 0)};
  obstacle(map, 0, 0);
  EXPECT_FALSE(validateOrRestorePath(candidate, raw, map, footprint(), false));
  EXPECT_EQ(candidate, raw);
  EXPECT_TRUE(validateOrRestorePath(candidate, raw, map, footprint(), false));
  obstacle(map, 0, 1);
  EXPECT_THROW(validateOrRestorePath(candidate, raw, map, footprint(), false),
    nav2_core::NoValidPathCouldBeFound);
}

TEST(FinalPath, RejectsActualSmootherRearCornerRegression)
{
  nav2_costmap_2d::Costmap2D map(160, 160, .05, -4, -4, 0);
  nav2_smac_planner::SmootherParams params;
  params.tolerance_ = 1e-10; params.max_its_ = 1000;
  params.w_data_ = .2; params.w_smooth_ = .3;
  params.holonomic_ = false; params.do_refinement_ = true; params.refinement_num_ = 2;
  nav2_smac_planner::Smoother smoother(params); smoother.initialize(.5);
  nav_msgs::msg::Path path;
  for (int i = 0; i <= 20; ++i) {path.poses.push_back(pose((i - 20) * .05, 0));}
  for (int i = 1; i <= 20; ++i) {path.poses.push_back(pose(0, i * .05, M_PI / 2));}
  // Reproduced in the review with a free center but colliding smoothed rear corner.
  for (unsigned y = 0; y < 160; ++y) {
    for (unsigned x = 0; x < 160; ++x) {
      double wx, wy; map.mapToWorld(x, y, wx, wy);
      const double d = std::hypot(wx + .275, wy + .425);
      map.setCost(x, y, d < 1e-5 ? 254 : d <= .11 ? 253 : d <= .45 ?
        static_cast<unsigned char>(252 * std::exp(-3 * (d - .11))) : 0);
    }
  }
  EXPECT_TRUE(smoother.smooth(path, &map, 5.0));
  EXPECT_FALSE(isPathCollisionFree(path, map, footprint(), false));
}

TEST(GridBounds, RejectsExclusiveUpperBoundsAndNonfiniteCoordinates)
{
  nav2_smac_planner::GridCollisionChecker checker(nullptr, 72, nullptr);
  nav2_costmap_2d::Costmap2D map(20, 20, .05, 0, 0, 0);
  checker.setCostmap(&map);
  EXPECT_FALSE(checker.outsideRange(20, 19.999f));
  for (float v : {-1.0f, 20.0f, 21.0f, std::numeric_limits<float>::infinity(),
      std::numeric_limits<float>::quiet_NaN()})
  {
    EXPECT_TRUE(checker.outsideRange(20, v));
    EXPECT_TRUE(checker.inCollision(v, 0.0f, 0.0f, false));
    EXPECT_TRUE(checker.inCollision(0.0f, v, 0.0f, false));
  }
  EXPECT_TRUE(checker.inCollision(400u, false));
}
