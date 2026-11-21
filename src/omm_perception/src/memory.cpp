#include "omm_perception/memory.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace omm_perception {
namespace {
void execute(sqlite3 *db, const char *sql) {
  if (sqlite3_exec(db, sql, nullptr, nullptr, nullptr) != SQLITE_OK)
    throw std::runtime_error(sqlite3_errmsg(db));
}
struct Statement {
  sqlite3_stmt *p = nullptr;
  Statement(sqlite3 *db, const char *sql) {
    if (sqlite3_prepare_v2(db, sql, -1, &p, nullptr) != SQLITE_OK)
      throw std::runtime_error(sqlite3_errmsg(db));
  }
  bool row() {
    const int result = sqlite3_step(p);
    if (result == SQLITE_ROW) return true;
    if (result == SQLITE_DONE) return false;
    throw std::runtime_error(sqlite3_errmsg(sqlite3_db_handle(p)));
  }
  ~Statement() { sqlite3_finalize(p); }
  Statement(const Statement &) = delete;
  Statement &operator=(const Statement &) = delete;
};
struct Transaction {
  sqlite3 *db;
  bool committed = false;
  explicit Transaction(sqlite3 *database) : db(database) { execute(db, "BEGIN"); }
  ~Transaction() {
    if (!committed)
      sqlite3_exec(db, "ROLLBACK", nullptr, nullptr, nullptr);
  }
  void commit() { execute(db, "COMMIT"); committed = true; }
};
std::array<int, 3> keyOf(const Eigen::Vector3d &p, double size) {
  return {int(std::floor(p.x() / size)), int(std::floor(p.y() / size)),
          int(std::floor(p.z() / size))};
}
int floorDivide(int x, int n) { return x >= 0 ? x / n : -((-x + n - 1) / n); }
} // namespace
size_t KeyHash::operator()(const std::array<int, 3> &k) const {
  size_t h = 0;
  for (int n : k)
    h ^= std::hash<int>{}(n) + 0x9e3779b9 + (h << 6) + (h >> 2);
  return h;
}
Visibility visibility(const Eigen::Vector3d &p, const cv::Mat &depth,
                      const Intrinsics &k, double margin, double near,
                      double far) {
  if (!p.allFinite() || p.z() <= near || p.z() >= far)
    return Visibility::Outside;
  double u = k.fx * p.x() / p.z() + k.cx, v = k.fy * p.y() / p.z() + k.cy;
  if (!std::isfinite(u) || !std::isfinite(v) || u < 2 || v < 2 ||
      u >= depth.cols - 2 || v >= depth.rows - 2)
    return Visibility::Outside;
  int x = std::lround(u), y = std::lround(v);
  bool all_free = true, closer = false;
  for (int yy = y - 1; yy <= y + 1; ++yy)
    for (int xx = x - 1; xx <= x + 1; ++xx) {
      float z = depth.at<float>(yy, xx);
      if (!std::isfinite(z) || z <= near || z >= far)
        return Visibility::Uncertain;
      all_free &= z > p.z() + margin;
      closer |= z < p.z() - margin;
    }
  if (all_free)
    return Visibility::Free;
  return closer ? Visibility::Occluded : Visibility::Visible;
}
Memory::Memory(double size, size_t max) : voxel_size_(size), max_voxels_(max) {
  if (!std::isfinite(size) || size <= 0 || !max)
    throw std::invalid_argument("Invalid memory size");
  // Empty filename creates a private temporary on-disk DB, deleted on close.
  // Never reopen old odom-coordinate memory in a new simulation session.
  if (sqlite3_open("", &archive_) != SQLITE_OK)
    throw std::runtime_error("Cannot create occupancy archive");
  execute(
      archive_,
      "PRAGMA cache_size=-2048; PRAGMA max_page_count=32768; "
      "PRAGMA journal_mode=MEMORY; CREATE TABLE "
      "voxels(kx INTEGER,ky INTEGER,kz INTEGER,x REAL,y REAL,z REAL,stamp "
      "REAL,free INTEGER,owner INTEGER,last_free_frame INTEGER,PRIMARY KEY(kx,ky,kz)); CREATE INDEX "
      "voxel_xy ON voxels(x,y); CREATE INDEX voxel_owner ON voxels(owner);");
}
Memory::~Memory() {
  if (archive_)
    sqlite3_close(archive_);
}
void Memory::archiveAdded(const std::array<int, 3> &key, double z) {
  auto &bin = archive_bins_[{floorDivide(key[0], archive_factor_),
                             floorDivide(key[1], archive_factor_), 0}];
  if (!bin.count)
    bin.min_z = z;
  else
    bin.min_z = std::min(bin.min_z, z);
  ++bin.count;
  ++archive_count_;
  compactArchive();
}
void Memory::archiveRemoved(const std::array<int, 3> &key) {
  auto it = archive_bins_.find({floorDivide(key[0], archive_factor_),
                                floorDivide(key[1], archive_factor_), 0});
  if (it == archive_bins_.end() || !it->second.count)
    throw std::runtime_error("Occupancy archive index mismatch");
  if (--it->second.count == 0)
    archive_bins_.erase(it);
  --archive_count_;
}
void Memory::compactArchive() const {
  while (archive_bins_.size() > archive_budget_) {
    std::unordered_map<std::array<int, 3>, ArchiveBin, KeyHash> coarse;
    for (const auto &[key, value] : archive_bins_) {
      auto &bin = coarse[{floorDivide(key[0], 2), floorDivide(key[1], 2), 0}];
      if (!bin.count)
        bin.min_z = value.min_z;
      else
        bin.min_z = std::min(bin.min_z, value.min_z);
      bin.count += value.count;
    }
    archive_bins_ = std::move(coarse);
    archive_factor_ *= 2;
  }
}
void Memory::store(const std::array<int, 3> &key, const Voxel &v,
                   sqlite3_stmt *statement) {
  // The caller owns one prepared statement for the whole transaction. Reset
  // before rebinding; finalization on every exception is handled by Statement.
  if (sqlite3_reset(statement) != SQLITE_OK)
    throw std::runtime_error(sqlite3_errmsg(archive_));
  for (int i = 0; i < 3; ++i) {
    sqlite3_bind_int(statement, i + 1, key[i]);
    sqlite3_bind_double(statement, i + 4, v.point[i]);
  }
  sqlite3_bind_double(statement, 7, v.last_seen);
  sqlite3_bind_int(statement, 8, v.free_frames);
  sqlite3_bind_int64(statement, 9, v.owner);
  sqlite3_bind_int64(statement, 10, v.last_free_frame);
  if (sqlite3_step(statement) != SQLITE_DONE)
    throw std::runtime_error(sqlite3_errmsg(archive_));

}
bool Memory::observe(const std::vector<Eigen::Vector3d> &points, double stamp,
                     uint64_t owner) {
  std::unordered_map<std::array<int, 3>, std::pair<Eigen::Vector3d, int>,
                     KeyHash>
      frame;
  for (const auto &p : points) {
    if (!p.allFinite() || p.cwiseAbs().maxCoeff() > 10000.)
      continue;
    auto [it, fresh] = frame.try_emplace(
        keyOf(p, voxel_size_), std::make_pair(Eigen::Vector3d::Zero(), 0));
    (void)fresh;
    it->second.first += p;
    ++it->second.second;
  }
  Statement erase(archive_, "DELETE FROM voxels WHERE kx=? AND ky=? AND kz=?");
  Statement previous(archive_,
                     "SELECT owner FROM voxels WHERE kx=? AND ky=? AND kz=?");
  std::vector<std::pair<std::array<int, 3>, Voxel>> active;
  std::vector<std::array<int, 3>> removed;
  active.reserve(frame.size());
  const bool has_archive = archivedCount() > 0;
  Transaction transaction(archive_);
  for (const auto &[key, value] : frame) {
    auto old = voxels_.find(key);
    uint64_t assigned = owner;
    if (old != voxels_.end() && old->second.owner != owner)
      assigned = 0; // Preserve independently supported static occupancy.
    if (has_archive && old == voxels_.end()) {
      for (int i = 0; i < 3; ++i)
        sqlite3_bind_int(previous.p, i + 1, key[i]);
      if (previous.row()) {
        if (uint64_t(sqlite3_column_int64(previous.p, 0)) != owner)
          assigned = 0;
        removed.push_back(key);
      }
      sqlite3_reset(previous.p);
    }
    active.emplace_back(key, Voxel{value.first / value.second, stamp, 0,
                                        Visibility::Visible, assigned});
    if (has_archive) {
      for (int i = 0; i < 3; ++i)
        sqlite3_bind_int(erase.p, i + 1, key[i]);
      if (sqlite3_step(erase.p) != SQLITE_DONE)
        throw std::runtime_error(sqlite3_errmsg(archive_));
      sqlite3_reset(erase.p);
    }
  }
  transaction.commit();
  for (const auto &key : removed) archiveRemoved(key);
  for (const auto &[key,v] : active) voxels_.insert_or_assign(key,v);
  // Overflow is archived, not silently discarded or reported as sensor failure.
  if (voxels_.size() > max_voxels_)
    recenter(center_, active_radius_);
  return true;
}
void Memory::recenter(const Eigen::Vector3d &center, double radius) {
  std::vector<std::array<int, 3>> evicted;
  std::vector<std::pair<std::array<int, 3>, Voxel>> restored;
  std::vector<std::pair<double, std::array<int, 3>>> ordered;
  ordered.reserve(voxels_.size());
  for (const auto &[key, v] : voxels_)
    ordered.push_back({(v.point - center).head<2>().squaredNorm(), key});
  if (ordered.size() > max_voxels_)
    std::sort(ordered.begin(), ordered.end(),
              [](const auto &a, const auto &b) { return a.first < b.first; });
  Transaction transaction(archive_);
  Statement insert(archive_,
                   "INSERT OR REPLACE INTO voxels VALUES(?,?,?,?,?,?,?,?,?,?)");
  for (size_t i = 0; i < ordered.size(); ++i)
    if (i >= max_voxels_ || ordered[i].first > radius * radius) {
      auto it = voxels_.find(ordered[i].second);
      store(it->first, it->second, insert.p);
      evicted.push_back(it->first);
    }
  // Reload precise observations on re-entry. Archived areas were never labelled
  // free.
  const size_t available = max_voxels_ - (voxels_.size() - evicted.size());
  if (available && archive_count_) {
    Statement s(archive_,
                "SELECT kx,ky,kz,x,y,z,stamp,free,owner,last_free_frame FROM voxels WHERE x "
                "BETWEEN ?1 AND ?2 AND y BETWEEN ?3 AND ?4 AND "
                "(x-?5)*(x-?5)+(y-?6)*(y-?6) <= ?7 "
                "ORDER BY (x-?5)*(x-?5)+(y-?6)*(y-?6),kx,ky,kz LIMIT ?8");
    sqlite3_bind_double(s.p, 1, center.x() - radius);
    sqlite3_bind_double(s.p, 2, center.x() + radius);
    sqlite3_bind_double(s.p, 3, center.y() - radius);
    sqlite3_bind_double(s.p, 4, center.y() + radius);
    sqlite3_bind_double(s.p, 5, center.x());
    sqlite3_bind_double(s.p, 6, center.y());
    sqlite3_bind_double(s.p, 7, radius * radius);
    sqlite3_bind_int64(s.p, 8, available);
    std::vector<std::array<int, 3>> loaded;
    while (s.row()) {
      Eigen::Vector3d p(sqlite3_column_double(s.p, 3),
                        sqlite3_column_double(s.p, 4),
                        sqlite3_column_double(s.p, 5));
      std::array<int, 3> key{sqlite3_column_int(s.p, 0),
                             sqlite3_column_int(s.p, 1),
                             sqlite3_column_int(s.p, 2)};
      const auto last_free = uint64_t(sqlite3_column_int64(s.p, 9));
      const int votes = last_free == clear_frame_ ? sqlite3_column_int(s.p, 7) : 0;
      restored.emplace_back(
          key, Voxel{p, sqlite3_column_double(s.p, 6), votes, Visibility::Uncertain,
                     uint64_t(sqlite3_column_int64(s.p, 8)), last_free});
      loaded.push_back(key);
    }
    Statement erase(archive_,
                    "DELETE FROM voxels WHERE kx=? AND ky=? AND kz=?");
    for (const auto &key : loaded) {
      for (int i = 0; i < 3; ++i)
        sqlite3_bind_int(erase.p, i + 1, key[i]);
      if (sqlite3_step(erase.p) != SQLITE_DONE)
        throw std::runtime_error(sqlite3_errmsg(archive_));
      sqlite3_reset(erase.p);

    }
  }
  transaction.commit();
  center_ = center;
  active_radius_ = radius;
  for (const auto &key : evicted) {
    archiveAdded(key, voxels_.at(key).point.z());
    voxels_.erase(key);
  }
  for (const auto &[key, voxel] : restored) {
    archiveRemoved(key);
    voxels_.insert_or_assign(key, voxel);
  }
}
size_t Memory::archivedCount() const { return archive_count_; }
std::pair<double, std::vector<Eigen::Vector3d>>
Memory::archiveProjection(size_t max_cells) const {
  archive_budget_ = std::max(size_t(4), max_cells);
  if (archive_factor_ > 2 && archive_count_ <= archive_budget_ / 2) {
    decltype(archive_bins_) fine;
    Statement rows(archive_, "SELECT kx,ky,z FROM voxels");
    while (rows.row()) {
      auto &bin = fine[{floorDivide(sqlite3_column_int(rows.p,0),2),
                        floorDivide(sqlite3_column_int(rows.p,1),2),0}];
      double z = sqlite3_column_double(rows.p,2);
      bin.min_z = bin.count ? std::min(bin.min_z,z) : z;
      ++bin.count;
    }
    archive_bins_.swap(fine);
    archive_factor_ = 2;
  }
  compactArchive();
  std::vector<Eigen::Vector3d> points;
  points.reserve(archive_bins_.size());
  for (const auto &[key, bin] : archive_bins_)
    points.emplace_back((key[0] + .5) * archive_factor_ * voxel_size_,
                        (key[1] + .5) * archive_factor_ * voxel_size_,
                        bin.min_z);
  return {archive_factor_ * voxel_size_, points};
}
void Memory::removeOwner(uint64_t owner) {
  if (!owner)
    return;
  std::vector<std::array<int, 3>> removed;
  Transaction transaction(archive_);
  Statement query(archive_, "SELECT kx,ky,kz FROM voxels WHERE owner=?");
  sqlite3_bind_int64(query.p, 1, owner);
  while (query.row())
    removed.push_back({sqlite3_column_int(query.p, 0),
                    sqlite3_column_int(query.p, 1),
                    sqlite3_column_int(query.p, 2)});
  Statement s(archive_, "DELETE FROM voxels WHERE owner=?");
  sqlite3_bind_int64(s.p, 1, owner);
  if (sqlite3_step(s.p) != SQLITE_DONE)
    throw std::runtime_error(sqlite3_errmsg(archive_));
  transaction.commit();
  for (const auto &key : removed) archiveRemoved(key);
  for (auto it = voxels_.begin(); it != voxels_.end();)
    if (it->second.owner == owner)
      it = voxels_.erase(it);
    else
      ++it;
}
void Memory::clearVisible(const cv::Mat &depth, const Intrinsics &k,
                          const Eigen::Isometry3d &camera_from_memory,
                          double stamp, double margin, int frames, double near,
                          double far, double max_gap) {
  // One captured image can provide at most one free-space vote. Clock epochs
  // are reset by the node before accepting observations in the new session.
  if (!std::isfinite(stamp) || stamp <= last_clear_stamp_)
    return;
  const auto frame = clear_frame_ + (stamp - last_clear_stamp_ > max_gap ? 2 : 1);
  auto update = [&](Voxel &v) {
    if (v.last_seen == stamp)
      return false;
    v.visibility =
        visibility(camera_from_memory * v.point, depth, k, margin, near, far);
    if (v.visibility == Visibility::Free) {
      v.free_frames = v.last_free_frame + 1 == frame ? v.free_frames + 1 : 1;
      v.last_free_frame = frame;
    } else {
      v.free_frames = 0;
      v.last_free_frame = 0;
    }
    return v.free_frames >= frames && stamp - v.last_seen >= .2;
  };
  Eigen::Vector3d camera = camera_from_memory.inverse().translation();
  Statement s(archive_, "SELECT kx,ky,kz,x,y,z,stamp,free,owner,last_free_frame FROM voxels "
                        "WHERE x BETWEEN ? AND ? AND y BETWEEN ? AND ?");
  sqlite3_bind_double(s.p, 1, camera.x() - far);
  sqlite3_bind_double(s.p, 2, camera.x() + far);
  sqlite3_bind_double(s.p, 3, camera.y() - far);
  sqlite3_bind_double(s.p, 4, camera.y() + far);
  std::vector<std::pair<std::array<int, 3>, Voxel>> retained;
  std::vector<std::array<int, 3>> deleted;
  while (s.row()) {
    std::array<int, 3> key{sqlite3_column_int(s.p, 0),
                           sqlite3_column_int(s.p, 1),
                           sqlite3_column_int(s.p, 2)};
    Voxel v{{sqlite3_column_double(s.p, 3), sqlite3_column_double(s.p, 4),
             sqlite3_column_double(s.p, 5)},
            sqlite3_column_double(s.p, 6),
            sqlite3_column_int(s.p, 7),
            Visibility::Uncertain,
            uint64_t(sqlite3_column_int64(s.p, 8)),
            uint64_t(sqlite3_column_int64(s.p, 9))};
    const auto previous_free_frame = v.last_free_frame;
    if (update(v)) {
      deleted.push_back(key);

    } else if (previous_free_frame != v.last_free_frame)
      retained.push_back({key, v});
  }
  Transaction transaction(archive_);
  Statement insert(archive_,
                   "INSERT OR REPLACE INTO voxels VALUES(?,?,?,?,?,?,?,?,?,?)");
  for (const auto &[key, v] : retained)
    store(key, v, insert.p);
  Statement erase(archive_, "DELETE FROM voxels WHERE kx=? AND ky=? AND kz=?");
  for (const auto &key : deleted) {
    for (int i = 0; i < 3; ++i)
      sqlite3_bind_int(erase.p, i + 1, key[i]);
    if (sqlite3_step(erase.p) != SQLITE_DONE)
      throw std::runtime_error(sqlite3_errmsg(archive_));
    sqlite3_reset(erase.p);

  }
  transaction.commit();
  last_clear_stamp_ = stamp;
  clear_frame_ = frame;
  for (const auto &key : deleted) archiveRemoved(key);
  cleared_ += deleted.size();
  for (auto it = voxels_.begin(); it != voxels_.end();)
    if (update(it->second)) {
      it = voxels_.erase(it);
      ++cleared_;
    } else
      ++it;

}
void Memory::reset() {
  execute(archive_, "DELETE FROM voxels");
  voxels_.clear();
  archive_bins_.clear();
  archive_count_ = 0;
  archive_factor_ = 2;
  cleared_ = clear_frame_ = 0;
  last_clear_stamp_ = -std::numeric_limits<double>::infinity();
  center_.setZero();
}
} // namespace omm_perception
