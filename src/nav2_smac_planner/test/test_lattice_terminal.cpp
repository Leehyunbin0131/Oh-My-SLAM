// Copyright 2026 Oh_My_SLAM contributors. Apache-2.0.
#include <cmath>
#include <limits>
#include <memory>
#include <random>
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
      rclcpp::Parameter("plugins", std::vector<std::string>{"inflation_layer"}),
      rclcpp::Parameter("inflation_layer.plugin", "nav2_costmap_2d::InflationLayer"),
      rclcpp::Parameter("inflation_layer.inflation_radius", 0.55),
      rclcpp::Parameter("inflation_layer.cost_scaling_factor", 3.0),
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

  void configure(double tolerance = 0.1, const std::string & model = "diff", bool smooth = true)
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
      rclcpp::Parameter("test.smooth_path", smooth),
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

TEST_F(TerminalPlanner, UsesClearRotationArcAndRejectsTwoBlockedArcs)
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
  auto alternate = plan(start, goal);
  ASSERT_TRUE(nav2_smac_planner::isPathCollisionFree(
    alternate, *costmap_ros->getCostmap(), costmap_ros->getRobotFootprint(), false));
  EXPECT_EQ(alternate.poses.front().pose, start.pose);
  EXPECT_EQ(alternate.poses.back().pose, goal.pose);
  obstacle(0.33, 0.33, nav2_costmap_2d::LETHAL_OBSTACLE);
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

TEST_F(TerminalPlanner, GeneralPathChecksAndRetainsMeasuredStart)
{
  const auto start = pose(0.0, 0.0, 0.20);
  const auto goal = pose(0.8, 0.0);
  for (bool smooth : {false, true}) {
    configure(0.1, "diff", smooth);
    obstacle(-0.325, -0.375, nav2_costmap_2d::LETHAL_OBSTACLE);
    nav_msgs::msg::Path measured;
    measured.poses.push_back(start);
    ASSERT_FALSE(nav2_smac_planner::isPathCollisionFree(
      measured, *costmap_ros->getCostmap(), costmap_ros->getRobotFootprint(), false));
    EXPECT_THROW(plan(start, goal), nav2_core::StartOccupied);
    obstacle(-0.325, -0.375, nav2_costmap_2d::FREE_SPACE);
    const auto route = plan(start, goal);
    ASSERT_FALSE(route.poses.empty());
    EXPECT_EQ(route.poses.front().pose, start.pose);
    EXPECT_EQ(route.poses.back().pose, goal.pose);
    EXPECT_TRUE(nav2_smac_planner::isPathCollisionFree(
      route, *costmap_ros->getCostmap(), costmap_ros->getRobotFootprint(), false));
  }
}

TEST_F(TerminalPlanner, ReplansAroundAnUninflatedUnknownBoundary)
{
  auto * grid = costmap_ros->getCostmap();
  grid->resizeMap(241, 202, 0.05, -1.01705, -1.06441);
  for (unsigned int y = 0; y < grid->getSizeInCellsY(); ++y) {
    for (unsigned int x = 0; x < grid->getSizeInCellsX(); ++x) {
      double wx, wy;
      grid->mapToWorld(x, y, wx, wy);
      grid->setCost(x, y, wx > 3.6 && wy < -0.52 ?
        nav2_costmap_2d::NO_INFORMATION : nav2_costmap_2d::FREE_SPACE);
    }
  }
  // The straight aisle and a final in-place turn are free. A forward-curving
  // shortcut can keep its centre in the aisle while its body enters unknown.
  const auto start = pose(0.90732, 0.0000616, 0.012142);
  const auto goal = pose(5.0, 0.0, M_PI_2);
  for (bool smooth : {false, true}) {
    configure(0.1, "diff", smooth);
    const auto route = plan(start, goal);
    ASSERT_FALSE(route.poses.empty());
    EXPECT_EQ(route.poses.front().pose, start.pose);
    EXPECT_EQ(route.poses.back().pose, goal.pose);
    EXPECT_TRUE(nav2_smac_planner::isPathCollisionFree(
      route, *grid, costmap_ros->getRobotFootprint(), false));
  }
}

TEST_F(TerminalPlanner, SearchSweepMatchesIndependentFilledFootprintCheck)
{
  auto * grid = costmap_ros->getCostmap();
  const auto footprint = costmap_ros->getRobotFootprint();
  // Separate the centre-only inscribed cost from lethal/unknown cells at
  // corners, inside the body, across a rotation, and next to the map edge.
  const unsigned char costs[] = {253, 254, 255};
  for (unsigned int y = 4; y < 86; y += 13) {
    for (unsigned int x = 7; x < 99; x += 19) {
      grid->setCost(x, y, costs[(x + y) % 3]);
    }
  }
  auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>("sweep_comparison");
  nav2_smac_planner::GridCollisionChecker checker(costmap_ros, 72, parent);
  checker.setFootprint(footprint, false, 100.0);
  for (bool allow_unknown : {false, true}) {
    checker.prepareLatticeFootprint(allow_unknown);
    std::mt19937 engine(17);
    std::uniform_real_distribution<float> location(0, 115), offset(-4, 4), angle(-M_PI, M_PI);
    unsigned int blocked = 0, clear = 0;
    for (unsigned int i = 0; i < 320; ++i) {
      const float x = location(engine), y = location(engine), a = angle(engine);
      const float nx = x + offset(engine), ny = y + offset(engine), b = angle(engine);
      nav_msgs::msg::Path edge;
      edge.poses.resize(2);
      edge.poses[0].pose = nav2_smac_planner::getWorldCoords(x, y, grid);
      edge.poses[0].pose.orientation = nav2_smac_planner::getWorldOrientation(a);
      edge.poses[1].pose = nav2_smac_planner::getWorldCoords(nx, ny, grid);
      edge.poses[1].pose.orientation = nav2_smac_planner::getWorldOrientation(b);
      const bool expected = !nav2_smac_planner::isPathCollisionFree(edge, *grid, footprint, allow_unknown);
      ASSERT_EQ(checker.latticeSweepCollides(x, y, a, nx, ny, b), expected) << "case " << i;
      if (expected) {++blocked;} else {++clear;}
    }
    EXPECT_GT(blocked, 0u);
    EXPECT_GT(clear, 0u);
  }
}

TEST_F(TerminalPlanner, IncomingPrimitivesHaveIndependentCollisionAndCost)
{
  configure();
  using nav2_smac_planner::NodeLattice;
  // Trigger initialization of the actual diff motion table for this costmap.
  ASSERT_FALSE(plan(pose(0, 0), pose(0.8, 0)).poses.empty());
  nav2_smac_planner::GridCollisionChecker checker(costmap_ros, 72, node);
  checker.setFootprint({}, true, 0.0);
  auto & straight = NodeLattice::motion_table.motion_primitives.at(0).at(3);
  auto & turning = NodeLattice::motion_table.motion_primitives.at(1).at(2);
  ASSERT_EQ(straight.trajectory_id, 3u);
  ASSERT_EQ(turning.trajectory_id, 9u);
  for (auto cost : {nav2_costmap_2d::LETHAL_OBSTACLE, static_cast<unsigned char>(100)}) {
    costmap_ros->getCostmap()->setCost(54, 58, cost);
    for (bool turn_first : {false, true}) {
      NodeLattice destination(0);
      destination.setPose({60.5f, 60.5f, 0.0f});
      for (bool turn : {turn_first, !turn_first}) {
        EXPECT_EQ(destination.isNodeValid(false, &checker, turn ? &turning : &straight),
          !turn || cost != nav2_costmap_2d::LETHAL_OBSTACLE);
        EXPECT_EQ(destination.getCost(), turn ? cost : 0);
      }
    }
  }
}

TEST_F(TerminalPlanner, AllHeadingDistancesReflectAcrossTheGoalAxis)
{
  configure();
  ASSERT_FALSE(plan(pose(0, 0), pose(0.8, 0)).poses.empty());
  using nav2_smac_planner::NodeHybrid;
  using nav2_smac_planner::NodeLattice;
  unsigned int size = 160, bins = NodeLattice::motion_table.num_angle_quantization;
  nav2_smac_planner::SearchInfo info;
  NodeHybrid::initMotionModel(nav2_smac_planner::MotionModel::DUBIN, size, size, bins, info);
  NodeHybrid::precomputeDistanceHeuristic(31, nav2_smac_planner::MotionModel::DUBIN, bins, info);
  const NodeLattice::Coordinates goal(60, 60, 0);
  for (unsigned int bin = 0; bin < bins; ++bin) {
    const NodeLattice::Coordinates left(50, 62, bin), right(50, 58, (bins - bin) % bins);
    EXPECT_FLOAT_EQ(NodeLattice::getDistanceHeuristic(left, goal, 0),
      NodeLattice::getDistanceHeuristic(right, goal, 0));
    EXPECT_FLOAT_EQ(NodeHybrid::getDistanceHeuristic(left, goal, 0),
      NodeHybrid::getDistanceHeuristic(right, goal, 0));
  }
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
