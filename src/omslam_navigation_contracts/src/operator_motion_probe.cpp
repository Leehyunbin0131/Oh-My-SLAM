// Copyright 2026 Oh My SLAM contributors. Apache-2.0.
// Exercise the configured Nav2 local controller in-process on a synthetic empty map.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "dwb_core/dwb_local_planner.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "omslam_operator_msgs/msg/operator_state.hpp"
#include "pluginlib/class_loader.hpp"
#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace
{
void require(bool ok, const char * message)
{
  if (!ok) {throw std::runtime_error(message);}
}

geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2);
  p.pose.orientation.w = std::cos(yaw / 2);
  return p;
}

void run_probe()
{
  using State = omslam_operator_msgs::msg::OperatorState;
  auto parent = std::make_shared<nav2_util::LifecycleNode>("controller_server", "",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).use_intra_process_comms(true));
  rclcpp::NodeOptions cm_options;
  cm_options.parameter_overrides({
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("global_frame", "map"), rclcpp::Parameter("resolution", 0.05),
    rclcpp::Parameter("footprint", "[[0.09,0.26],[0.09,-0.26],[-0.39,-0.26],[-0.39,0.26]]"),
    rclcpp::Parameter("footprint_padding", 0.02)});
  auto cm = std::make_shared<nav2_costmap_2d::Costmap2DROS>(cm_options);
  cm->on_configure(rclcpp_lifecycle::State());
  auto grid = cm->getCostmap();
  grid->resizeMap(160, 160, 0.05, -4.0, -4.0);
  std::fill_n(grid->getCharMap(), 160 * 160, nav2_costmap_2d::FREE_SPACE);
  pluginlib::ClassLoader<nav2_core::GoalChecker> goals("nav2_core", "nav2_core::GoalChecker");
  pluginlib::ClassLoader<nav2_core::ProgressChecker> checks("nav2_core", "nav2_core::ProgressChecker");
  auto goal = goals.createSharedInstance(parent->get_parameter("goal_checker.plugin").as_string());
  auto progress = checks.createSharedInstance(parent->get_parameter("progress_checker.plugin").as_string());
  goal->initialize(parent, "goal_checker", cm);
  progress->initialize(parent, "progress_checker");
  const double inside = parent->get_parameter("goal_checker.xy_goal_tolerance").as_double() * 0.9;
  const auto target = pose(0, 0, 0);
  geometry_msgs::msg::Twist stopped;
  require(!goal->isGoalReached(pose(inside, 0, .5).pose, target.pose, stopped), "wrong yaw accepted");
  require(!goal->isGoalReached(pose(.13, 0, 0).pose, target.pose, stopped), "outside XY accepted after drift");
  require(goal->isGoalReached(pose(inside, 0, 0).pose, target.pose, stopped), "realigned goal rejected");
  auto moving = stopped;
  moving.linear.x = .08;
  require(!goal->isGoalReached(pose(inside, 0, 0).pose, target.pose, moving), "moving goal accepted");
  std::cout << "arrival_inside_outside_inside=false,false,true; moving=false\n";

  dwb_core::DWBLocalPlanner dwb;
  auto tf = std::make_shared<tf2_ros::Buffer>(parent->get_clock());
  dwb.configure(parent, "FollowPath", tf, cm);
  dwb.activate();
  auto pub = parent->create_publisher<State>("/operator/state", rclcpp::QoS(1).transient_local());
  pub->on_activate();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(parent->get_node_base_interface());
  auto clock = parent->get_clock()->get_clock_handle();
  int64_t now = 10000000000LL;
  require(rcl_enable_ros_time_override(clock) == RCL_RET_OK, "cannot enable offline clock");
  auto advance = [&](int64_t duration) {
      now += duration;
      require(rcl_set_ros_time_override(clock, now) == RCL_RET_OK, "offline clock update failed");
    };

  for (double rpm : {1.0, 2.0}) {
    State limits;
    limits.operator_enabled = true;
    limits.max_linear_velocity = parent->get_parameter("FollowPath.max_speed_xy").as_double();
    limits.max_angular_velocity = parent->get_parameter("FollowPath.max_vel_theta").as_double();
    const double full_rpm = limits.max_linear_velocity * 30.0 / (M_PI * 0.068);
    limits.speed_percent = 100.0 * rpm / full_rpm;
    pub->publish(limits);
    exec.spin_some();
    dwb.setSpeedLimit(limits.speed_percent, true);
    for (bool rotate : {false, true}) {
      nav_msgs::msg::Path plan;
      plan.header.frame_id = "map";
      for (int i = 0; i <= 40; ++i) {
        plan.poses.push_back(pose(rotate ? 0.0 : .05 * i, 0.0, rotate ? .03 * i : 0.0));
      }
      progress->reset();
      goal->reset();
      auto p = pose(0, 0, 0);
      geometry_msgs::msg::Twist vel;
      double x = 0.0, y = 0.0, heading = 0.0;
      for (int step = 0; step < 410; ++step) {
        advance(50000000);
        p.header.stamp = parent->now();
        if (step % 20 == 0) {dwb.setPlan(plan);}
        require(progress->check(p), "moving robot failed the progress check");
        vel = dwb.computeVelocityCommands(p, vel, goal.get()).twist;
        const double vmax = limits.max_linear_velocity * limits.speed_percent / 100.0;
        const double wmax = limits.max_angular_velocity * limits.speed_percent / 100.0;
        require(std::abs(vel.linear.x) <= vmax + 1e-9 && std::abs(vel.angular.z) <= wmax + 1e-9,
          "DWB command exceeded its approved cap");
        x += vel.linear.x * .05 * std::cos(heading);
        y += vel.linear.x * .05 * std::sin(heading);
        heading += vel.angular.z * .05;
        p = pose(x, y, heading);
      }
      require(rotate ? std::abs(x) < 1e-9 && heading > .06 * rpm : x > .072 * rpm,
        "trajectory made insufficient physical progress");
      progress->reset();
      require(progress->check(p), "new progress baseline rejected");
      advance(10100000000LL);
      require(!progress->check(p), "stationary robot passed the stall deadline");
      std::cout << "rpm=" << rpm << " rotation=" << rotate << " x_m=" << x <<
        " yaw_rad=" << heading << " stall_at_10.1s=true\n";
    }
  }
  exec.remove_node(parent->get_node_base_interface());
  dwb.deactivate();
  dwb.cleanup();
  cm->on_cleanup(rclcpp_lifecycle::State());
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    run_probe();
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
