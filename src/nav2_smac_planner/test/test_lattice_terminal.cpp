// Copyright 2026 Oh_My_SLAM contributors. Apache-2.0.
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_smac_planner/path_validation.hpp"
#include "nav2_smac_planner/smac_planner_lattice.hpp"

namespace
{
class RclcppLifetime
{
public:
  RclcppLifetime() {rclcpp::init(0, nullptr);}
  ~RclcppLifetime() {rclcpp::shutdown();}
} rclcpp_lifetime;

geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2);
  p.pose.orientation.w = std::cos(yaw / 2);
  return p;
}

double length(const nav_msgs::msg::Path & path)
{
  double total = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    total += std::hypot(b.x - a.x, b.y - a.y);
  }
  return total;
}

class TerminalPlanner : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("plugins", std::vector<std::string>{}),
      rclcpp::Parameter("footprint", "[[0.09,0.26],[0.09,-0.26],[-0.39,-0.26],[-0.39,0.26]]"),
      rclcpp::Parameter("footprint_padding", 0.02),
      rclcpp::Parameter("resolution", 0.05)});
    costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    costmap_ros->on_configure(rclcpp_lifecycle::State());
    costmap_ros->getCostmap()->resizeMap(160, 160, 0.05, -4.0, -4.0);
    costmap_ros->getCostmap()->resetMap(0, 0, 160, 160);
    // Empty map is the causal control: no obstacle requires a near-goal loop.
    auto * map = costmap_ros->getCostmap();
    std::fill(map->getCharMap(), map->getCharMap() + 160 * 160, 0);
  }

  void configure(double tolerance = 0.1, const std::string & model = "diff")
  {
    if (planner) {
      if (active) {
        planner->deactivate();
        planner->cleanup();
      }
      planner.reset();
      node.reset();
    }
    active = false;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("test.xy_goal_tolerance", tolerance),
      rclcpp::Parameter("test.tolerance", 0.1),
      rclcpp::Parameter("test.smooth_path", true),
      rclcpp::Parameter("test.allow_unknown", false),
      rclcpp::Parameter("test.max_planning_time", 2.0),
      rclcpp::Parameter("test.lattice_filepath",
      ament_index_cpp::get_package_share_directory("nav2_smac_planner") +
      "/sample_primitives/5cm_resolution/0.5m_turning_radius/" + model + "/output.json")});
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("near_goal_test", options);
    planner = std::make_unique<nav2_smac_planner::SmacPlannerLattice>();
    planner->configure(node, "test", nullptr, costmap_ros);
    planner->activate();
    active = true;
  }

  void TearDown() override
  {
    if (planner && active) {
      planner->deactivate();
      planner->cleanup();
    }
    planner.reset();
    node.reset();
    costmap_ros->on_cleanup(rclcpp_lifecycle::State());
    costmap_ros.reset();
  }

  nav_msgs::msg::Path plan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    return planner->createPlan(start, goal, [] {return false;});
  }

  void obstacle(double x, double y, unsigned char cost)
  {
    unsigned int mx, my;
    ASSERT_TRUE(costmap_ros->getCostmap()->worldToMap(x, y, mx, my));
    costmap_ros->getCostmap()->setCost(mx, my, cost);
  }

  bool active{false};
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros;
  std::unique_ptr<nav2_smac_planner::SmacPlannerLattice> planner;
};

TEST_F(TerminalPlanner, NearbyDifferentCellsPreserveExactGoalWithoutLoop)
{
  configure();
  // Relative geometry from the reported 5.8 cm loop, on a synthetic empty grid.
  const auto start = pose(-0.03852649614515968, 1.8274873444354398, M_PI / 2);
  const auto goal = pose(-0.0742801575058305, 1.8732784965855442, 1.5225619693390453);
  const auto path = plan(start, goal);
  ASSERT_EQ(path.poses.size(), 3u);
  EXPECT_EQ(path.poses.front().pose, start.pose);
  EXPECT_EQ(path.poses[1].pose.position, start.pose.position);
  EXPECT_EQ(path.poses[1].pose.orientation, goal.pose.orientation);
  EXPECT_EQ(path.poses.back().pose, goal.pose);
  EXPECT_NEAR(length(path), 0.058096075, 1e-8);
  EXPECT_TRUE(nav2_smac_planner::isPathCollisionFree(
    path, *costmap_ros->getCostmap(), costmap_ros->getRobotFootprint(), false));
}

TEST_F(TerminalPlanner, InclusiveBoundaryAndOutsideWindowKeepSearch)
{
  configure();
  const auto start = pose(0, 0);
  const auto boundary = pose(0.1, 0, 0.2);
  const auto inside = plan(start, boundary);
  ASSERT_EQ(inside.poses.size(), 3u);
  EXPECT_EQ(inside.poses.back().pose, boundary.pose);
  const auto outside_goal = pose(0.1001, 0);
  const auto outside = plan(start, outside_goal);
  configure(0.0);
  const auto ordinary = plan(start, outside_goal);
  ASSERT_EQ(outside.poses.size(), ordinary.poses.size() + 1);
  for (size_t i = 0; i < ordinary.poses.size(); ++i) {
    EXPECT_EQ(outside.poses[i].pose, ordinary.poses[i].pose);
  }
  EXPECT_EQ(outside.poses.back().pose, outside_goal.pose);
}

TEST_F(TerminalPlanner, ApproachAndSameCellPreserveRequestedEndpoint)
{
  configure();
  const auto start = pose(0.01784652868168024, 1.7048303296682548, 2.03444387);
  const auto goal = pose(-0.0742801575058305, 1.8732784965855442, 1.5225619693390453);
  const auto approach = plan(start, goal);
  EXPECT_GT(approach.poses.size(), 3u);
  EXPECT_EQ(approach.poses.back().pose, goal.pose);
  // Outside the terminal window, preserve the original search/smoothing. Its
  // geometry depends on the grid origin; a synthetic grid cannot inherit the
  // recorded map's short-path length just by reusing the relative goal poses.
  configure(0.0);
  const auto ordinary = plan(start, goal);
  ASSERT_EQ(approach.poses.size(), ordinary.poses.size() + 1);
  for (size_t i = 0; i < ordinary.poses.size(); ++i) {
    EXPECT_EQ(approach.poses[i].pose, ordinary.poses[i].pose);
  }

  // A tighter XY tolerance does not permit the same-cell shortcut to move the
  // requested endpoint to the robot, even when outside the arrival window.
  configure(0.001);
  const auto same_cell_goal = pose(0.02, 0.02, 0.1);
  const auto same_cell = plan(pose(0.01, 0.01), same_cell_goal);
  EXPECT_EQ(same_cell.poses.back().pose, same_cell_goal.pose);
}

TEST_F(TerminalPlanner, RejectsSweptRotationEvenWithFreeEndpoints)
{
  configure();
  const auto start = pose(0, 0);
  const auto goal = pose(0.06, 0, M_PI);
  obstacle(0.33, -0.33, nav2_costmap_2d::LETHAL_OBSTACLE);
  for (const auto & endpoint : {start, pose(0, 0, M_PI), goal}) {
    nav_msgs::msg::Path single;
    single.poses.push_back(endpoint);
    ASSERT_TRUE(nav2_smac_planner::isPathCollisionFree(
      single, *costmap_ros->getCostmap(), costmap_ros->getRobotFootprint(), false));
  }
  EXPECT_THROW(plan(start, goal), nav2_core::NoValidPathCouldBeFound);
}

TEST_F(TerminalPlanner, RejectsUnknownFootprintAndHonorsCancellation)
{
  configure();
  const auto start = pose(0, 0);
  const auto goal = pose(0.06, 0);
  obstacle(-0.30, 0.10, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_THROW(plan(start, goal), nav2_core::NoValidPathCouldBeFound);
  obstacle(-0.30, 0.10, nav2_costmap_2d::FREE_SPACE);
  EXPECT_THROW(planner->createPlan(start, goal, [] {return true;}), nav2_core::PlannerCancelled);
  unsigned int checks = 0;
  EXPECT_THROW(planner->createPlan(start, goal, [&checks] {return ++checks == 2;}),
    nav2_core::PlannerCancelled);
}

TEST_F(TerminalPlanner, StartupToleranceIsReadOnly)
{
  configure();
  EXPECT_TRUE(node->describe_parameter("test.xy_goal_tolerance").read_only);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("test.xy_goal_tolerance", 0.2)).successful);
  EXPECT_DOUBLE_EQ(node->get_parameter("test.xy_goal_tolerance").as_double(), 0.1);
}

TEST_F(TerminalPlanner, RejectsInvalidTolerance)
{
  EXPECT_THROW(configure(-0.1), std::invalid_argument);
}

TEST_F(TerminalPlanner, RejectsNonfiniteTolerance)
{
  EXPECT_THROW(configure(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

TEST_F(TerminalPlanner, RejectsNonRotatingMotionModel)
{
  EXPECT_THROW(configure(0.1, "ackermann"), std::invalid_argument);
}
}  // namespace
