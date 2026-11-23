// Copyright 2026 Oh My SLAM contributors. Apache-2.0.
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"

namespace omslam_navigation
{
struct DriveGeometry
{
  double left_radius, right_radius, separation, min_rpm, max_rpm;

  static DriveGeometry read(
    const nav2_util::LifecycleNode::SharedPtr & node, const std::string & plugin)
  {
    const auto param = [&](const std::string & name) {
        const auto key = plugin + ".drive_geometry." + name;
        nav2_util::declare_parameter_if_not_declared(node, key, rclcpp::PARAMETER_DOUBLE);
        return node->get_parameter(key).as_double();
      };
    const DriveGeometry geometry{
      param("left_radius"), param("right_radius"), param("wheel_separation"),
      param("min_rpm"), param("max_rpm")};
    for (const double value : {geometry.left_radius, geometry.right_radius, geometry.separation,
      geometry.min_rpm, geometry.max_rpm})
    {
      if (!std::isfinite(value) || value <= 0.) {
        throw std::invalid_argument("Drive geometry and RPM limits must be finite and positive");
      }
    }
    if (geometry.min_rpm >= geometry.max_rpm || geometry.max_rpm > 100.) {
      throw std::invalid_argument("Invalid drive RPM range");
    }
    return geometry;
  }

  double peakRpm(double vx, double wz) const
  {
    const double turn = wz * separation / 2.;
    const double left = (vx - turn) / left_radius * 30. / M_PI;
    const double right = (vx + turn) / right_radius * 30. / M_PI;
    return std::max(std::abs(left), std::abs(right));
  }

  bool atFloor(double rpm) const
  {
    return std::abs(rpm - min_rpm) <=
           8. * (std::nextafter(min_rpm, std::numeric_limits<double>::infinity()) - min_rpm);
  }

  double straightFloor() const {return min_rpm / peakRpm(1., 0.);}
  double rotationFloor() const {return min_rpm / peakRpm(0., 1.);}

  nav_2d_msgs::msg::Twist2D applied(nav_2d_msgs::msg::Twist2D twist, double cap) const
  {
    const double peak = peakRpm(twist.x, twist.theta);
    if (peak == 0. || cap < min_rpm || (peak < min_rpm && !atFloor(peak))) {
      return nav_2d_msgs::msg::Twist2D{};
    }
    const double scale = std::clamp(peak, min_rpm, cap) / peak;
    twist.x *= scale;
    twist.theta *= scale;
    return twist;
  }
};
}  // namespace omslam_navigation
