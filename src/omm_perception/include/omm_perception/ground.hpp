#pragma once
#include <Eigen/Geometry>
#include <vector>
namespace omm_perception {
struct GroundPlane {
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  double offset = 0.;
  size_t support = 0;
  bool valid = false;
  double height(const Eigen::Vector3d &p) const {
    return normal.dot(p) + offset;
  }
};
GroundPlane estimateGround(const std::vector<Eigen::Vector3d> &points,
                           const Eigen::Vector3d &up, double max_slope = .35,
                           const GroundPlane *previous = nullptr);
} // namespace omm_perception
