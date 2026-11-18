// Copyright 2026 Oh My SLAM contributors. Apache-2.0.
// Calls the installed planner and DWB footprint critic with a center-free,
// corner-colliding costmap. There is no motion output or robot connection.
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
#include "nav2_core/global_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_ros/buffer.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 1;
  try {
    if (argc < 2) {throw std::runtime_error("Need absolute lattice filepath argument");}
    // Keep plugin libraries loaded until every lifecycle callback is destroyed.
    pluginlib::ClassLoader<nav2_core::GlobalPlanner> planner_loader("nav2_core","nav2_core::GlobalPlanner");
    pluginlib::ClassLoader<dwb_core::TrajectoryCritic> critic_loader("dwb_core","dwb_core::TrajectoryCritic");
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
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("p4_fixture", false);
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
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
    auto grid = costmap->getCostmap();
    grid->resizeMap(120,120,0.05,-2.0,-2.0);
    grid->resetMap(0,0,120,120);
    for (unsigned y=0;y<120;++y) {
      for (unsigned x=0;x<120;++x) {
        double wx,wy; grid->mapToWorld(x,y,wx,wy);
        grid->setCost(x,y, wx>=0.5 && (std::abs(wy)>=0.25 || wx>=3.0) ?
          nav2_costmap_2d::LETHAL_OBSTACLE : nav2_costmap_2d::FREE_SPACE);
      }
    }
    unsigned mx,my; grid->worldToMap(2.0,0.0,mx,my);
    bool center_free=grid->getCost(mx,my)==nav2_costmap_2d::FREE_SPACE;
    auto tf=std::make_shared<tf2_ros::Buffer>(parent->get_clock());
    auto planner=planner_loader.createSharedInstance("nav2_smac_planner::SmacPlannerLattice");
    planner->configure(parent,"GridBased",tf,costmap); planner->activate();
    geometry_msgs::msg::PoseStamped start,goal;
    start.header.frame_id=goal.header.frame_id="map";
    start.pose.orientation.w=goal.pose.orientation.w=1.0;
    goal.pose.position.x=2.0;
    bool planner_rejected=false;
    try {planner_rejected=planner->createPlan(start,goal,[]{return false;}).poses.empty();}
    catch (const nav2_core::GoalOccupied & e) {planner_rejected=true; std::cerr<<"Goal footprint rejected: "<<e.what()<<'\n';}
    catch (const nav2_core::NoValidPathCouldBeFound & e) {planner_rejected=true; std::cerr<<"No valid footprint path: "<<e.what()<<'\n';}
    goal.pose.position.x=0.0; goal.pose.position.y=0.7;
    bool planner_control=!planner->createPlan(start,goal,[]{return false;}).poses.empty();
    auto critic=critic_loader.createSharedInstance("dwb_critics::ObstacleFootprintCritic");
    critic->initialize(parent,"ObstacleFootprint","FollowPath",costmap);
    geometry_msgs::msg::Pose2D pose; nav_2d_msgs::msg::Twist2D velocity;
    nav_2d_msgs::msg::Path2D path;
    critic->prepare(pose,velocity,pose,path);
    dwb_msgs::msg::Trajectory2D trajectory;
    pose.x=1.5; pose.y=0.0; pose.theta=0.0; trajectory.poses.push_back(pose);
    pose.x=1.6; trajectory.poses.push_back(pose);
    bool candidate_rejected=false;
    try {(void)critic->scoreTrajectory(trajectory);}
    catch(const dwb_core::IllegalTrajectoryException &) {candidate_rejected=true;}
    trajectory.poses.clear(); pose.x=0.0; trajectory.poses.push_back(pose);
    bool critic_control=std::isfinite(critic->scoreTrajectory(trajectory));
    // The recorded 007 origin makes these nearby continuous poses straddle a
    // cell boundary. Both have a legal full footprint and straight primitive.
    grid->resizeMap(120,120,0.05,-2.0,-1.03266627181);
    for (unsigned y=0;y<120;++y) {
      for (unsigned x=0;x<120;++x) {
        double wx,wy; grid->mapToWorld(x,y,wx,wy);
        grid->setCost(x,y, std::abs(wy)>=0.34 ?
          nav2_costmap_2d::LETHAL_OBSTACLE : nav2_costmap_2d::FREE_SPACE);
      }
    }
    auto valid_plan = [&](double y) {
        start.pose.position.x=0.0; start.pose.position.y=y;
        goal.pose.position.x=1.5; goal.pose.position.y=y;
        try {return !planner->createPlan(start,goal,[]{return false;}).poses.empty();}
        catch (const nav2_core::StartOccupied &) {return false;}
        catch (const nav2_core::GoalOccupied &) {return false;}
        catch (const nav2_core::NoValidPathCouldBeFound &) {return false;}
      };
    const bool boundary_above_valid=valid_plan(-0.032612);
    const bool boundary_below_valid=valid_plan(-0.03267487);
    const bool actual_footprint_collision_rejected=!valid_plan(-0.08);

    // A tiny footprint isolates center indexing: it remains entirely in the
    // free cell, so the adjacent unknown cell cannot replace its center cost.
    grid->resetMapToValue(0,0,120,120,nav2_costmap_2d::FREE_SPACE);
    grid->setCost(61,30,nav2_costmap_2d::NO_INFORMATION);
    nav2_smac_planner::GridCollisionChecker checker(costmap,72,parent);
    nav2_costmap_2d::Footprint tiny;
    for (const auto & xy : std::vector<std::pair<double,double>>{
        {.001,.001},{.001,-.001},{-.001,-.001},{-.001,.001}}) {
      geometry_msgs::msg::Point p; p.x=xy.first; p.y=xy.second; tiny.push_back(p);
    }
    checker.setFootprint(tiny,false,0.0);
    const bool continuous_center_index_valid=!checker.inCollision(60.7f,30.2f,0.0f,false);
    grid->setCost(60,30,nav2_costmap_2d::NO_INFORMATION);
    const bool continuous_center_unknown_rejected=checker.inCollision(60.7f,30.2f,0.0f,false);
    std::cout<<"P4_BOUNDARY_RESULT {\"center_free\":"<<(center_free?"true":"false")
      <<",\"planner_rejected\":"<<(planner_rejected?"true":"false")
      <<",\"planner_free_control\":"<<(planner_control?"true":"false")
      <<",\"dwb_candidate_rejected\":"<<(candidate_rejected?"true":"false")
      <<",\"dwb_free_control\":"<<(critic_control?"true":"false")
      <<",\"boundary_above_valid\":"<<(boundary_above_valid?"true":"false")
      <<",\"boundary_below_valid\":"<<(boundary_below_valid?"true":"false")
      <<",\"actual_footprint_collision_rejected\":"<<(actual_footprint_collision_rejected?"true":"false")
      <<",\"continuous_center_index_valid\":"<<(continuous_center_index_valid?"true":"false")
      <<",\"continuous_center_unknown_rejected\":"<<(continuous_center_unknown_rejected?"true":"false")<<"}"<<std::endl;
    code=center_free&&planner_rejected&&planner_control&&candidate_rejected&&critic_control&&
      boundary_above_valid&&boundary_below_valid&&actual_footprint_collision_rejected&&
      continuous_center_index_valid&&continuous_center_unknown_rejected?0:1;
    critic.reset(); planner->deactivate(); planner->cleanup(); planner.reset();
    costmap->on_cleanup(rclcpp_lifecycle::State());
  } catch(const std::exception & e) {std::cerr<<"Fixture error: "<<e.what()<<'\n';}
  rclcpp::shutdown();
  return code;
}
