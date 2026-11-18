#include "omm_perception/tracker.hpp"
#include "omm_perception/memory.hpp"
#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
namespace omm_perception {
std::vector<Cluster>
clusterObstacles(const std::vector<Eigen::Vector3d> &points) {
  using Key = std::array<int, 3>;
  std::unordered_map<Key, std::vector<Eigen::Vector3d>, KeyHash> cells;
  for (const auto &p : points)
    cells[{int(std::floor(p.x() / .10)), int(std::floor(p.y() / .10)), 0}]
        .push_back(p);
  std::vector<Cluster> result;
  while (!cells.empty()) {
    std::queue<Key> queue;
    queue.push(cells.begin()->first);
    Cluster c;
    while (!queue.empty()) {
      auto k = queue.front();
      queue.pop();
      auto it = cells.find(k);
      if (it == cells.end())
        continue;
      c.points.insert(c.points.end(), it->second.begin(), it->second.end());
      cells.erase(it);
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx)
          if (dx || dy)
            queue.push({k[0] + dx, k[1] + dy, 0});
    }
    Eigen::Vector3d lo = c.points[0], hi = lo;
    for (const auto &p : c.points) {
      lo = lo.cwiseMin(p);
      hi = hi.cwiseMax(p);
    }
    c.center = (lo + hi) * .5;
    c.extent = hi - lo;
    result.push_back(std::move(c));
  }
  return result;
}
void Tracker::update(const std::vector<Cluster> &clusters, double stamp) {
  struct Match {
    double cost;
    size_t track, cluster;
  };
  std::vector<Match> matches;
  std::vector<bool> used_track(tracks_.size(), false),
      used_cluster(clusters.size(), false);
  for (size_t i = 0; i < tracks_.size(); ++i) {
    auto &t = tracks_[i];
    double dt = stamp - t.last_seen;
    if (dt <= 0 || (!t.stationary && dt > 1.5))
      continue;
    for (size_t j = 0; j < clusters.size(); ++j) {
      const auto &c = clusters[j];
      if (c.extent.head<2>().maxCoeff() > 1.6 || c.points.size() < 3)
        continue;
      double prediction = t.stationary && !t.moving ? 0. : dt;
      double distance = (c.center - t.cluster.center - t.velocity * prediction)
                            .head<2>()
                            .norm();
      double size_error = (c.extent - t.cluster.extent).head<2>().norm();
      if (distance < .20 + 1.5 * std::min(dt, .3) && size_error < .55)
        matches.push_back({distance + .3 * size_error, i, j});
    }
  }
  std::sort(matches.begin(), matches.end(),
            [](const auto &a, const auto &b) { return a.cost < b.cost; });
  for (const auto &m : matches) {
    if (used_track[m.track] || used_cluster[m.cluster])
      continue;
    auto &t = tracks_[m.track];
    const auto &c = clusters[m.cluster];
    double dt = stamp - t.last_seen;
    Eigen::Vector3d measured = (c.center - t.cluster.center) / dt;
    measured.z() = 0;
    t.velocity = .55 * t.velocity + .45 * measured;
    bool translated = (c.center - t.anchor).head<2>().norm() > .14;
    const bool stable_extent = (c.extent - t.anchor_extent).head<2>().norm() <
                               .08 + .1 * t.anchor_extent.head<2>().norm();
    int thin_axis = t.anchor_extent.x() < t.anchor_extent.y() ? 0 : 1;
    // The camera obstacle band is 0.02..0.30 m above the floor. A 0.35 m
    // height requirement can never recognize its clipped wall fragments.
    // Retain normal-motion evidence for these vertical surfaces: a changing
    // visible window slides their centroid along a stationary wall. The
    // 0.10 m low obstacles remain below this vertical-surface threshold.
    const bool planar_wall =
        t.anchor_extent.z() > .20 && t.anchor_extent[thin_axis] < .06;
    const bool normal_translation =
        !planar_wall ||
        std::abs(c.center[thin_axis] - t.anchor[thin_axis]) > .10;
    if (translated)
      t.stationary = false;
    t.motion_votes = translated && stable_extent && normal_translation &&
                             t.velocity.head<2>().norm() > .10
                         ? t.motion_votes + 1
                         : 0;
    if (t.motion_votes >= 3) {
      t.moving = true;
      t.stationary = false;
    }
    if (!t.moving && stamp - t.created >= .8 &&
        (c.center - t.anchor).head<2>().norm() < .10)
      t.stationary = true;
    if (t.velocity.head<2>().norm() < .05) {
      if (t.still_since < 0)
        t.still_since = stamp;
      if (!t.stationary && stamp - t.still_since > 1.) {
        t.moving = false;
        t.stationary = true;
        t.anchor = c.center;
        t.anchor_extent = c.extent;
        t.motion_votes = 0;
      }
    } else
      t.still_since = -1.;
    t.cluster = c;
    t.last_seen = stamp;
    ++t.observations;
    used_track[m.track] = true;
    used_cluster[m.cluster] = true;
  }
  for (size_t j = 0; j < clusters.size(); ++j)
    if (!used_cluster[j]) {
      const auto &c = clusters[j];
      if (c.extent.head<2>().maxCoeff() > 1.6 || c.points.size() < 3)
        continue;
      tracks_.push_back(Track{next_id_++, c, c.center, Eigen::Vector3d::Zero(),
                              stamp, stamp});
      tracks_.back().anchor_extent = c.extent;
    }
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [stamp](const Track &t) {
                                 return (!t.stationary || t.moving) &&
                                        stamp - t.last_seen > 2.;
                               }),
                tracks_.end());
  for (auto &t : tracks_)
    if (t.stationary && !t.moving && stamp - t.last_seen > 2.)
      t.cluster.points.clear();
  // Retire the oldest inactive association metadata, never its static
  // occupancy.
  if (tracks_.size() > 4096) {
    std::stable_sort(tracks_.begin(), tracks_.end(),
                     [](const Track &a, const Track &b) {
                       return a.last_seen > b.last_seen;
                     });
    tracks_.resize(4096);
  }
}
bool Tracker::inMovingRegion(const Eigen::Vector3d &point, double stamp) const {
  for (const auto &t : tracks_)
    if (t.moving) {
      double dt = stamp - t.last_seen;
      if (dt < 0 || dt > 1.)
        continue;
      Eigen::Vector3d delta =
          (point - t.cluster.center - t.velocity * dt).cwiseAbs();
      if (delta.x() < t.cluster.extent.x() / 2 + .15 &&
          delta.y() < t.cluster.extent.y() / 2 + .15)
        return true;
    }
  return false;
}
std::vector<Eigen::Vector3d> Tracker::dynamicPoints(double now,
                                                    double coast) const {
  std::vector<Eigen::Vector3d> points;
  for (const auto &t : tracks_) {
    double dt = now - t.last_seen;
    if (dt < 0 || dt > coast)
      continue;
    if (t.stationary && !t.moving)
      continue;
    Eigen::Vector3d shift = Eigen::Vector3d::Zero();
    if (t.moving)
      shift = t.velocity * dt;
    for (const auto &p : t.cluster.points)
      points.push_back(p + shift);
    if (t.moving && dt > .15) {
      double uncertainty = .04 + .15 * dt;
      for (double dx : {-uncertainty, uncertainty})
        for (double dy : {-uncertainty, uncertainty})
          for (const auto &p : t.cluster.points)
            points.push_back(p + shift + Eigen::Vector3d(dx, dy, 0));
    }
  }
  return points;
}
} // namespace omm_perception
