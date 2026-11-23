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
#include "dwb_core/trajectory_generator.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "omslam_operator_msgs/msg/operator_state.hpp"
#include "pluginlib/class_loader.hpp"
#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "drive_geometry.hpp"

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
  {
    auto expired = checks.createSharedInstance("omslam_navigation::SpeedAwareProgressChecker");
    bool rejected = false;
    try {
      expired->initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr{}, "expired_progress_checker");
    } catch (const std::runtime_error & error) {
      const std::string message = error.what();
      rejected = message.find("SpeedAwareProgressChecker") != std::string::npos &&
        message.find("expired_progress_checker") != std::string::npos;
    }
    require(rejected, "expired progress-checker parent did not produce the expected error");
  }
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

  const auto geometry = omslam_navigation::DriveGeometry::read(parent, "FollowPath");
  require(geometry.min_rpm == 3. && geometry.max_rpm == 70., "unexpected installed simulation RPM range");
  pluginlib::ClassLoader<dwb_core::TrajectoryGenerator> generators(
    "dwb_core", "dwb_core::TrajectoryGenerator");
  auto generator = generators.createSharedInstance(
    parent->get_parameter("FollowPath.trajectory_generator_name").as_string());
  generator->initialize(parent, "FollowPath");
  for (double rpm : {geometry.min_rpm, 10.0}) {
    State limits;
    limits.operator_enabled = true;
    limits.max_linear_velocity = parent->get_parameter("FollowPath.max_speed_xy").as_double();
    limits.max_angular_velocity = parent->get_parameter("FollowPath.max_vel_theta").as_double();
    const double fraction = rpm / geometry.max_rpm;
    limits.speed_percent = 100.0 * fraction;
    limits.auto_linear_limit = limits.max_linear_velocity * fraction;
    limits.auto_angular_limit = std::min(limits.max_angular_velocity,
      std::max(limits.max_angular_velocity * fraction, geometry.rotationFloor()));
    pub->publish(limits);
    exec.spin_some();
    dwb.setSpeedLimit(limits.speed_percent, true);
    auto verify_command = [&](const nav_2d_msgs::msg::Twist2D & cmd) {
        const double peak = geometry.peakRpm(cmd.x, cmd.theta);
        require(cmd.y == 0. && std::abs(cmd.x) <= limits.auto_linear_limit + 1e-12 &&
          std::abs(cmd.theta) <= limits.auto_angular_limit + 1e-12 && peak <= rpm + 1e-12,
          "DWB candidate exceeded its approved cap");
        require(peak == 0. || peak >= geometry.min_rpm || geometry.atFloor(peak),
          "DWB candidate cannot clear the command floor");
      };
    generator->setSpeedLimit(limits.speed_percent, true);
    generator->startNewIteration(nav_2d_msgs::msg::Twist2D{});
    int zeros = 0;
    bool straight = false, left = false, right = false;
    while (generator->hasMoreTwists()) {
      const auto cmd = generator->nextTwist();
      verify_command(cmd);
      const double peak = geometry.peakRpm(cmd.x, cmd.theta);
      zeros += peak == 0.;
      if (geometry.atFloor(peak)) {
        straight |= cmd.x > 0. && cmd.theta == 0.;
        left |= cmd.x == 0. && cmd.theta > 0.;
        right |= cmd.x == 0. && cmd.theta < 0.;
      }
    }
    require(zeros == 1 && straight && left && right, "trajectory grid lost a floor endpoint or explicit stop");
    const auto apply = [&](const geometry_msgs::msg::Twist & cmd) {
        nav_2d_msgs::msg::Twist2D twist;
        twist.x = cmd.linear.x;
        twist.y = cmd.linear.y;
        twist.theta = cmd.angular.z;
        verify_command(twist);
        const auto applied = geometry.applied(twist, rpm);
        geometry_msgs::msg::Twist output;
        output.linear.x = applied.x;
        output.angular.z = applied.theta;
        return output;
      };
    const auto step_pose = [&](geometry_msgs::msg::PoseStamped & p, double & heading,
        const geometry_msgs::msg::Twist & vel) {
        const double x = p.pose.position.x + vel.linear.x * .05 * std::cos(heading);
        const double y = p.pose.position.y + vel.linear.x * .05 * std::sin(heading);
        heading += vel.angular.z * .05;
        p = pose(x, y, heading);
      };
    for (int turn : {0, 1, -1}) {
      nav_msgs::msg::Path plan;
      plan.header.frame_id = "map";
      for (int i = 0; i <= 40; ++i) {
        plan.poses.push_back(pose(turn ? 0.0 : .05 * i, 0.0, turn * .03 * i));
      }
      progress->reset();
      goal->reset();
      auto p = pose(0, 0, 0);
      geometry_msgs::msg::Twist vel;
      double heading = 0.0;
      for (int step = 0; step < 100; ++step) {
        advance(50000000);
        p.header.stamp = parent->now();
        if (step % 20 == 0) {dwb.setPlan(plan);}
        require(progress->check(p), "moving robot failed the progress check");
        vel = apply(dwb.computeVelocityCommands(p, vel, goal.get()).twist);
        step_pose(p, heading, vel);
      }
      require(turn ? std::hypot(p.pose.position.x, p.pose.position.y) < 1e-9 && heading * turn > .25 :
        p.pose.position.x > 4. * limits.auto_linear_limit,
        "trajectory made insufficient physical progress");
      progress->reset();
      require(progress->check(p), "new progress baseline rejected");
      advance(10100000000LL);
      require(!progress->check(p), "stationary robot passed the stall deadline");
      std::cout << "rpm=" << rpm << " turn=" << turn << " x_m=" << p.pose.position.x <<
        " yaw_rad=" << heading << " stall_at_10.1s=true\n";
    }
    for (double sign : {1., -1.}) {
      for (bool already_inside : {false, true}) {
        const double distance = already_inside ? .049 : .051;
        const double yaw = sign * (already_inside ? .149 : .151);
        const auto target_pose = pose(distance, 0., yaw);
        nav_msgs::msg::Path plan;
        plan.header.frame_id = "map";
        if (!already_inside) {plan.poses.push_back(pose(0., 0., 0.));}
        plan.poses.insert(plan.poses.end(), {pose(distance, 0., 0.),
          pose(distance, 0., yaw / 2.), target_pose});
        dwb.setPlan(plan);
        goal->reset();
        progress->reset();
        auto p = pose(0., 0., 0.);
        double heading = 0.;
        geometry_msgs::msg::Twist vel;
        if (already_inside) {vel.linear.x = limits.auto_linear_limit;}
        require(!goal->isGoalReached(p.pose, target_pose.pose, vel), "arrival precondition was already reached");
        bool reached = false;
        for (int step = 0; step < 160; ++step) {
          advance(50000000);
          p.header.stamp = parent->now();
          require(progress->check(p), "near-goal command stalled");
          vel = apply(dwb.computeVelocityCommands(p, vel, goal.get()).twist);
          if (already_inside) {
            require(vel.linear.x == 0. && vel.angular.z == 0., "residual planned turn prevented braking");
          }
          step_pose(p, heading, vel);
          if (goal->isGoalReached(p.pose, target_pose.pose, vel)) {
            reached = true;
            break;
          }
        }
        const double xy = std::hypot(distance - p.pose.position.x, p.pose.position.y);
        const double yaw_error = std::abs(std::remainder(yaw - heading, 2. * M_PI));
        std::cout << "rpm=" << rpm << " arrival=" << (already_inside ? "inside" : "outside") <<
          " yaw_sign=" << sign << " xy_m=" << xy << " yaw_rad=" << yaw_error <<
          " vx=" << vel.linear.x << " wz=" << vel.angular.z << '\n';
        require(reached && xy <= .05 && yaw_error <= .15 &&
          std::abs(vel.linear.x) <= .01 && std::abs(vel.angular.z) <= .02,
          "failed to reach goal tolerance at measured stop");
      }
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
