#pragma once
#include <Eigen/Geometry>
#include <cstdint>
#include <vector>
namespace omm_perception {
struct Cluster {
  std::vector<Eigen::Vector3d> points;
  Eigen::Vector3d center, extent;
};
struct Track {
  uint64_t id;
  Cluster cluster;
  Eigen::Vector3d anchor, velocity = Eigen::Vector3d::Zero();
  double created, last_seen;
  int observations = 1, motion_votes = 0;
  bool moving = false, stationary = false;
  double still_since = -1.;
  Eigen::Vector3d anchor_extent = Eigen::Vector3d::Zero();
};
std::vector<Cluster>
clusterObstacles(const std::vector<Eigen::Vector3d> &points);
class Tracker {
public:
  void reset() { tracks_.clear(); next_id_ = 1; }
  void update(const std::vector<Cluster> &clusters, double stamp);
  const auto &tracks() const { return tracks_; }
  std::vector<Eigen::Vector3d> dynamicPoints(double now,
                                             double coast = .8) const;
  bool inMovingRegion(const Eigen::Vector3d &point, double stamp) const;

private:
  uint64_t next_id_ = 1;
  std::vector<Track> tracks_;
};
} // namespace omm_perception
