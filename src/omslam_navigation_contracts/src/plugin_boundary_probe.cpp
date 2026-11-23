// Copyright 2026 Oh My SLAM contributors. Apache-2.0.
// Loads the installed stock Smac Lattice planner and the production DWB
// filled-footprint critic through pluginlib. A real InflationLayer inflates a
// corridor whose center is free but whose walls touch the rectangular
// footprint. There is no motion output or robot connection.
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_ros/buffer.h"

namespace
{
const char * flag(bool value) {return value ? "true" : "false";}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 1;
  try {
    if (argc < 2) {throw std::runtime_error("Need absolute lattice filepath argument");}
    // Keep plugin libraries loaded until every lifecycle callback is destroyed.
    pluginlib::ClassLoader<nav2_core::GlobalPlanner> planner_loader("nav2_core", "nav2_core::GlobalPlanner");
    pluginlib::ClassLoader<dwb_core::TrajectoryCritic> critic_loader("dwb_core", "dwb_core::TrajectoryCritic");
    auto parent = std::make_shared<nav2_util::LifecycleNode>(
      "planner_server", "", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    // The verifier supplies the installed merged planner configuration. Do not
    // silently test different smoothing/tolerance/search defaults.
    if (!parent->has_parameter("GridBased.smooth_path") ||
      !parent->get_parameter("GridBased.smooth_path").as_bool() ||
      parent->get_parameter("GridBased.lattice_filepath").as_string() != argv[1])
    {
      throw std::runtime_error("Need production planner parameters with smoothing enabled");
    }
    if (parent->has_parameter("GridBased.xy_goal_tolerance")) {
      throw std::runtime_error("GridBased.xy_goal_tolerance is not a stock Jazzy Smac parameter");
    }

    // A real InflationLayer lets stock Smac compute its circumscribed cost and
    // check the full footprint wherever a center cell may be inscribed.
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("p4_fixture", false);
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{"inflation_layer"}));
    costmap->declare_parameter("inflation_layer.plugin", "nav2_costmap_2d::InflationLayer");
    costmap->declare_parameter("inflation_layer.inflation_radius", 0.55);
    costmap->declare_parameter("inflation_layer.cost_scaling_factor", 3.0);
    costmap->set_parameter(rclcpp::Parameter("global_frame", "map"));
    costmap->set_parameter(rclcpp::Parameter("robot_base_frame", "base_footprint"));
    costmap->set_parameter(rclcpp::Parameter("resolution", 0.05));
    costmap->set_parameter(rclcpp::Parameter("width", 6));
    costmap->set_parameter(rclcpp::Parameter("height", 6));
    costmap->set_parameter(rclcpp::Parameter("origin_x", -2.0));
    costmap->set_parameter(rclcpp::Parameter("origin_y", -2.0));
    costmap->set_parameter(rclcpp::Parameter("footprint_padding", 0.02));
    costmap->set_parameter(rclcpp::Parameter("footprint", "[[0.09,0.26],[0.09,-0.26],[-0.39,-0.26],[-0.39,0.26]]"));
    if (costmap->on_configure(rclcpp_lifecycle::State()) != nav2_util::CallbackReturn::SUCCESS) {
      throw std::runtime_error("Fixture costmap configure failed");
    }
    auto inflation = nav2_costmap_2d::InflationLayer::getInflationLayer(costmap);
    if (!inflation) {throw std::runtime_error("Fixture inflation layer was not loaded");}
    // Match every layer to the fixture grid before the lethal source is written.
    costmap->getLayeredCostmap()->resizeMap(120, 120, 0.05, -2.0, -2.0);
    auto grid = costmap->getCostmap();
    for (unsigned y = 0; y < 120; ++y) {
      for (unsigned x = 0; x < 120; ++x) {
        double wx, wy; grid->mapToWorld(x, y, wx, wy);
        grid->setCost(x, y, wx >= 0.5 && (std::abs(wy) >= 0.25 || wx >= 3.0) ?
          nav2_costmap_2d::LETHAL_OBSTACLE : nav2_costmap_2d::FREE_SPACE);
      }
    }
    // The layered costmap runs inflation over the lethal cells its sources wrote.
    inflation->updateCosts(*grid, 0, 0, grid->getSizeInCellsX(), grid->getSizeInCellsY());
    const double circumscribed = costmap->getLayeredCostmap()->getCircumscribedRadius();
    const unsigned possible_collision_cost = inflation->computeCost(circumscribed / grid->getResolution());
    unsigned mx, my; grid->worldToMap(2.0, 0.0, mx, my);
    const unsigned goal_center_cost = grid->getCost(mx, my);
    const bool inflation_layer_configured = inflation->getInflationRadius() >= circumscribed &&
      possible_collision_cost > nav2_costmap_2d::FREE_SPACE;
    // Free of the center's own collision, but inflated enough that stock Smac
    // must check the full rectangular footprint against the corridor walls.
    const bool goal_center_not_inscribed = goal_center_cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    const bool goal_center_requires_footprint_check = goal_center_cost >= possible_collision_cost;

    auto tf = std::make_shared<tf2_ros::Buffer>(parent->get_clock());
    auto planner = planner_loader.createSharedInstance("nav2_smac_planner::SmacPlannerLattice");
    planner->configure(parent, "GridBased", tf, costmap); planner->activate();
    geometry_msgs::msg::PoseStamped start, goal;
    start.header.frame_id = goal.header.frame_id = "map";
    start.pose.orientation.w = goal.pose.orientation.w = 1.0;
    goal.pose.position.x = 2.0;
    bool planner_rejected = false;
    try {planner_rejected = planner->createPlan(start, goal, [] {return false;}).poses.empty();}
    catch (const nav2_core::GoalOccupied & e) {planner_rejected = true; std::cerr << "Goal footprint rejected: " << e.what() << '\n';}
    catch (const nav2_core::NoValidPathCouldBeFound & e) {planner_rejected = true; std::cerr << "No valid footprint path: " << e.what() << '\n';}
    goal.pose.position.x = 0.0; goal.pose.position.y = 0.7;
    const bool planner_free_control = !planner->createPlan(start, goal, [] {return false;}).poses.empty();

    auto critic = critic_loader.createSharedInstance("omslam_navigation::FilledFootprintCritic");
    critic->initialize(parent, "ObstacleFootprint", "FollowPath", costmap);
    geometry_msgs::msg::Pose2D pose; nav_2d_msgs::msg::Twist2D velocity;
    nav_2d_msgs::msg::Path2D path;
    dwb_msgs::msg::Trajectory2D trajectory;
    pose.x = 0.0; pose.y = 0.0; pose.theta = 0.0;
    critic->prepare(pose, velocity, pose, path);
    pose.x = 1.5; trajectory.poses.push_back(pose);
    pose.x = 1.6; trajectory.poses.push_back(pose);
    bool dwb_candidate_rejected = false;
    try {(void)critic->scoreTrajectory(trajectory);}
    catch (const dwb_core::IllegalTrajectoryException &) {dwb_candidate_rejected = true;}
    pose.x = -0.5;
    critic->prepare(pose, velocity, pose, path);
    trajectory.poses.clear();
    pose.x = -0.4; trajectory.poses.push_back(pose);
    pose.x = -0.3; trajectory.poses.push_back(pose);
    const bool dwb_free_control = std::isfinite(critic->scoreTrajectory(trajectory));

    std::cout << "P4_BOUNDARY_RESULT {\"checks\":{"
      << "\"inflation_layer_configured\":" << flag(inflation_layer_configured)
      << ",\"goal_center_not_inscribed\":" << flag(goal_center_not_inscribed)
      << ",\"goal_center_requires_footprint_check\":" << flag(goal_center_requires_footprint_check)
      << ",\"planner_rejected\":" << flag(planner_rejected)
      << ",\"planner_free_control\":" << flag(planner_free_control)
      << ",\"dwb_candidate_rejected\":" << flag(dwb_candidate_rejected)
      << ",\"dwb_free_control\":" << flag(dwb_free_control)
      << "},\"evidence\":{\"goal_center_cost\":" << goal_center_cost
      << ",\"possible_collision_cost\":" << possible_collision_cost
      << ",\"circumscribed_radius\":" << circumscribed
      << ",\"inflation_radius\":" << inflation->getInflationRadius()
      << ",\"critic\":\"omslam_navigation::FilledFootprintCritic\"}}" << std::endl;
    code = inflation_layer_configured && goal_center_not_inscribed && goal_center_requires_footprint_check &&
      planner_rejected && planner_free_control && dwb_candidate_rejected && dwb_free_control ? 0 : 1;
    critic.reset(); planner->deactivate(); planner->cleanup(); planner.reset();
    inflation.reset();
    costmap->on_cleanup(rclcpp_lifecycle::State());
  } catch (const std::exception & e) {std::cerr << "Fixture error: " << e.what() << '\n';}
  rclcpp::shutdown();
  return code;
}
