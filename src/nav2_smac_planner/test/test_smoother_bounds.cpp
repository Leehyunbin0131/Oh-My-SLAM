// Copyright 2026 Oh_My_SLAM contributors
// Licensed under the Apache License, Version 2.0.

#include <cmath>
#include "gtest/gtest.h"
#include "nav2_smac_planner/smoother.hpp"

class BoundarySmoother : public nav2_smac_planner::Smoother
{
public:
  explicit BoundarySmoother(const nav2_smac_planner::SmootherParams & params)
  : Smoother(params) {}
  using Smoother::findBoundaryExpansion;
  using Smoother::smoothImpl;
};

TEST(SmootherBounds, RejectsCurveOutsideMapWithBothEndpointsInside)
{
  nav2_smac_planner::SmootherParams params;
  params.tolerance_ = 1e-10;
  params.max_its_ = 1000;
  params.w_data_ = 0.2;
  params.w_smooth_ = 0.3;
  params.do_refinement_ = false;
  params.refinement_num_ = 0;
  BoundarySmoother smoother(params);
  smoother.initialize(0.5);
  nav2_costmap_2d::Costmap2D map(20, 20, 0.05, 0.0, 0.0, 0);

  // Both poses are in free cells. Facing away from the destination forces
  // the interpolated Dubins curve outside this one-metre map.
  geometry_msgs::msg::Pose start, end;
  start.position.x = 0.1;
  start.position.y = 0.5;
  end.position.x = 0.9;
  end.position.y = 0.5;
  start.orientation.z = end.orientation.z = 1.0;
  nav2_smac_planner::BoundaryExpansion outside;
  outside.path_end_idx = 100;
  outside.original_path_length = 10.0;
  smoother.findBoundaryExpansion(start, end, outside, &map);
  EXPECT_TRUE(outside.in_collision);
  EXPECT_LT(outside.pts.size(), 101u);

  // The same endpoints facing along the straight connection stay valid.
  start.orientation.z = end.orientation.z = 0.0;
  start.orientation.w = end.orientation.w = 1.0;
  nav2_smac_planner::BoundaryExpansion inside;
  inside.path_end_idx = 100;
  inside.original_path_length = 0.8;
  smoother.findBoundaryExpansion(start, end, inside, &map);
  EXPECT_FALSE(inside.in_collision);
  EXPECT_EQ(inside.pts.size(), 101u);
  EXPECT_NEAR(inside.expansion_path_length, 0.8, 1e-6);
}

TEST(SmootherBounds, RejectsIterationOutsideMapBeforeReadingCost)
{
  nav2_smac_planner::SmootherParams params;
  params.tolerance_ = 1e-10; params.max_its_ = 1000;
  params.w_data_ = 0.2; params.w_smooth_ = 0.3;
  params.do_refinement_ = false; params.refinement_num_ = 0;
  BoundarySmoother smoother(params); smoother.initialize(0.5);
  nav2_costmap_2d::Costmap2D map(20, 20, 0.05, 0.0, 0.0, 0);
  // Out-of-map neighbors pull the otherwise valid center out on the first step.
  for (double outside : {-2.0, 3.0}) {
    nav_msgs::msg::Path path;
    for (double x : {outside, 0.5, outside}) {
      geometry_msgs::msg::PoseStamped p;
      p.pose.position.x = x; p.pose.position.y = 0.5;
      p.pose.orientation.w = 1.0; path.poses.push_back(p);
    }
    bool reversing = false;
    EXPECT_FALSE(smoother.smoothImpl(path, reversing, &map, 1.0));
    EXPECT_DOUBLE_EQ(path.poses[1].pose.position.x, 0.5);
  }
}
