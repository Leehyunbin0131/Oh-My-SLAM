#include "omm_perception/ground.hpp"
#include <Eigen/Eigenvalues>
#include <cmath>
#include <random>
namespace omm_perception {
GroundPlane estimateGround(const std::vector<Eigen::Vector3d> &points,
                           const Eigen::Vector3d &up, double slope) {
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
  std::mt19937 random(42);
  std::uniform_int_distribution<size_t> select(0, candidates.size() - 1);
  for (int trial = 0; trial < 90; ++trial) {
    auto a = candidates[select(random)], b = candidates[select(random)],
         c = candidates[select(random)];
    Eigen::Vector3d n = (b - a).cross(c - a);
    if (n.norm() < .003)
      continue;
    n.normalize();
    if (n.dot(up) < 0)
      n = -n;
    double offset = -n.dot(a);
    if (n.dot(up) < std::cos(slope) || std::abs(offset) > .04)
      continue;
    size_t support = 0;
    for (const auto &p : candidates)
      if (std::abs(n.dot(p) + offset) < .012)
        ++support;
    if (support > best.support)
      best = {n, offset, support, true};
  }
  if (best.support < 100 || best.support < candidates.size() * .10) {
    best.valid = false;
    return best;
  }
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> inliers;
  for (const auto &p : candidates)
    if (std::abs(best.height(p)) < .012) {
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
      best.normal.dot(up) >= std::cos(slope) && std::abs(best.offset) <= .04;
  return best;
}
} // namespace omm_perception
