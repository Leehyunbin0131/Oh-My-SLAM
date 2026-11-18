#include "omm_perception/ground.hpp"
#include <Eigen/Eigenvalues>
#include <cmath>
#include <random>
namespace omm_perception {
GroundPlane estimateGround(const std::vector<Eigen::Vector3d> &points,
                           const Eigen::Vector3d &up, double slope,
                           const GroundPlane *previous) {
  GroundPlane best;
  std::vector<Eigen::Vector3d> candidates;
  for (const auto &p : points)
    if (p.allFinite() && p.head<2>().norm() < 4. && p.dot(up) > -.4 &&
        p.dot(up) < .3)
      candidates.push_back(p);
  if (candidates.size() < 100)
    return best;
  if (candidates.size() > 2000) {
    std::vector<Eigen::Vector3d> sampled;
    sampled.reserve(2000);
    for (size_t i = 0; i < 2000; ++i)
      sampled.push_back(candidates[i * candidates.size() / 2000]);
    candidates = std::move(sampled);
  }
  // base_footprint is on the ground. Keep its height uncertainty below the
  // 2 cm obstacle band, and require support across the current image even
  // when the previous frame supplies a plane hypothesis.
  constexpr double origin_tolerance = .012;
  constexpr double inlier_tolerance = .010;
  const double min_up = std::cos(slope);
  auto consider = [&](Eigen::Vector3d normal, double offset) {
    if (normal.dot(up) < 0) {
      normal = -normal;
      offset = -offset;
    }
    if (normal.dot(up) < min_up || std::abs(offset) > origin_tolerance)
      return;
    size_t support = 0, below = 0;
    for (const auto &p : candidates) {
      const double height = normal.dot(p) + offset;
      support += std::abs(height) < inlier_tolerance;
      below += height < -inlier_tolerance;
    }
    // A coherent lower surface is evidence against the top of a step.
    if (below <= std::max(size_t(12), candidates.size() / 20) && support > best.support)
      best = {normal, offset, support, true};
  };
  consider(up, 0.);
  if (previous && previous->valid)
    consider(previous->normal, previous->offset);
  std::mt19937 random(42);
  std::uniform_int_distribution<size_t> select(0, candidates.size() - 1);
  for (int trial = 0; trial < 90; ++trial) {
    const auto &a = candidates[select(random)], &b = candidates[select(random)],
               &c = candidates[select(random)];
    Eigen::Vector3d n = (b - a).cross(c - a);
    if (n.norm() >= .003) {
      n.normalize();
      consider(n, -n.dot(a));
    }
    // Two observed floor points and the wheel contact plane also propose a
    // slope when the real floor accounts for only a small part of the image.
    n = a.cross(b);
    if (n.norm() >= .003)
      consider(n.normalized(), 0.);
  }
  if (best.support < 100 || best.support < candidates.size() * .10) {
    best.valid = false;
    return best;
  }
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> inliers;
  for (const auto &p : candidates)
    if (std::abs(best.height(p)) < inlier_tolerance) {
      mean += p;
      inliers.push_back(p);
    }
  mean /= inliers.size();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto &p : inliers)
    covariance += (p - mean) * (p - mean).transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance /
                                                        inliers.size());
  // A line or tiny horizontal patch is not enough evidence for a floor.
  if (solver.info() != Eigen::Success || solver.eigenvalues()[1] < .0004) {
    best.valid = false;
    return best;
  }
  best.normal = solver.eigenvectors().col(0);
  if (best.normal.dot(up) < 0)
    best.normal = -best.normal;
  best.offset = -best.normal.dot(mean);
  best.valid =
      best.normal.dot(up) >= min_up && std::abs(best.offset) <= origin_tolerance;
  return best;
}
} // namespace omm_perception
