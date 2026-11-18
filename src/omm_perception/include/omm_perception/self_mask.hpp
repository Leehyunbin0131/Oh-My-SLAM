#pragma once
#include <Eigen/Geometry>
#include <cmath>

namespace omm_perception {
// Collision primitives from omslam_description/upstream/urdf/base.urdf,
// expressed in base_footprint. Caster envelopes include their entire swivel.
// Use the collision geometry, not the Nav2 padded footprint: the latter would
// suppress real obstacles next to the robot.
inline bool insideRobot(const Eigen::Vector3d &p) {
  constexpr double margin = .01;
  if (!p.allFinite())
    return false;
  auto box = [&](const Eigen::Vector3d &center, const Eigen::Vector3d &half) {
    return ((p - center).cwiseAbs().array() <= half.array() + margin).all();
  };
  if (box({-.14, 0., .147}, {.203, .223, .100}) ||
      box({-.130999489541, 0., .254}, {.04699948954, .03799316171, .031}) ||
      box({.031055, .03875, .2045}, {.034055, .08375, .0125}))
    return true;
  for (double side : {-1., 1.}) {
    // Drive wheel cylinders have their axis along Y.
    if (std::abs(p.y() - side * .24) <= .012 + margin &&
        std::hypot(p.x(), p.z() - .068) <= .068 + margin)
      return true;
    // Swept caster wheel and fork envelopes about the vertical swivel axis.
    const double radial = std::hypot(p.x() + .30775, p.y() - side * .155);
    if (radial <= std::hypot(.034 + .038, .0165) + margin &&
        p.z() >= -margin && p.z() <= .076 + margin)
      return true;
    if (radial <= std::hypot(.00925 + .0715 / 2., .053 / 2.) + margin &&
        p.z() >= .026 - margin && p.z() <= .101 + margin)
      return true;
  }
  return false;
}
} // namespace omm_perception
