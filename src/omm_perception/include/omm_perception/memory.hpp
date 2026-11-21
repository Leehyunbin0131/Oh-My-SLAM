#pragma once
#include <Eigen/Geometry>
#include <array>
#include <limits>
#include <opencv2/core.hpp>
#include <sqlite3.h>
#include <unordered_map>
#include <vector>

namespace omm_perception {
enum class Visibility { Visible, Occluded, Outside, Uncertain, Free };
struct Intrinsics {
  double fx, fy, cx, cy;
};
struct Voxel {
  Eigen::Vector3d point;
  double last_seen;
  int free_frames = 0;
  Visibility visibility = Visibility::Visible;
  uint64_t owner = 0;
  uint64_t last_free_frame = 0;
};
struct KeyHash {
  size_t operator()(const std::array<int, 3> &k) const;
};
Visibility visibility(const Eigen::Vector3d &camera_point, const cv::Mat &depth,
                      const Intrinsics &k, double clear_margin,
                      double min_depth, double max_depth);
class Memory {
public:
  Memory(double voxel_size, size_t max_voxels);
  ~Memory();
  Memory(const Memory &) = delete;
  Memory &operator=(const Memory &) = delete;
  bool observe(const std::vector<Eigen::Vector3d> &points, double stamp,
               uint64_t owner = 0);
  void recenter(const Eigen::Vector3d &center, double radius);
  void removeOwner(uint64_t owner);
  size_t archivedCount() const;
  std::pair<double, std::vector<Eigen::Vector3d>>
  archiveProjection(size_t max_cells = 4096) const;
  void clearVisible(const cv::Mat &depth, const Intrinsics &k,
                    const Eigen::Isometry3d &camera_from_memory, double stamp,
                    double margin, int required_frames, double min_depth,
                    double max_depth, double max_gap = std::numeric_limits<double>::infinity());
  void interruptFreeEvidence() { ++clear_frame_; }
  void reset();
  const auto &voxels() const { return voxels_; }
  uint64_t cleared() const { return cleared_; }

private:
  double voxel_size_;
  size_t max_voxels_;
  uint64_t cleared_ = 0, clear_frame_ = 0;
  double last_clear_stamp_ = -std::numeric_limits<double>::infinity();
  sqlite3 *archive_ = nullptr;
  Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
  double active_radius_ = 6.;
  struct ArchiveBin {
    size_t count = 0;
    double min_z = 0.;
  };
  mutable std::unordered_map<std::array<int, 3>, ArchiveBin, KeyHash>
      archive_bins_;
  mutable int archive_factor_ = 2;
  mutable size_t archive_budget_ = 4096;
  size_t archive_count_ = 0;
  void store(const std::array<int, 3> &key, const Voxel &voxel,
             sqlite3_stmt *statement);
  void archiveAdded(const std::array<int, 3> &key, double z);
  void archiveRemoved(const std::array<int, 3> &key);
  void compactArchive() const;
  std::unordered_map<std::array<int, 3>, Voxel, KeyHash> voxels_;
};
} // namespace omm_perception
