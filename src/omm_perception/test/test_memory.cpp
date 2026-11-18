#include "omm_perception/memory.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace omm_perception;
namespace {
Intrinsics k{100, 100, 50, 50};
cv::Mat image(float z) {
  return cv::Mat(100, 100, CV_32FC1, cv::Scalar(z)).clone();
}
void clear(Memory &m, const cv::Mat &d, double t,
           const Eigen::Isometry3d &tf = Eigen::Isometry3d::Identity()) {
  m.clearVisible(d, k, tf, t, .1, 5, .12, 5.);
}
} // namespace
TEST(Memory, RepeatedFreeSpaceRequired) {
  Memory m(.05, 100);
  m.observe({{0, 0, 2}}, 1.);
  for (int i = 0; i < 4; ++i)
    clear(m, image(3), 1.1 + i * .1);
  EXPECT_EQ(m.voxels().size(), 1u);
  clear(m, image(3), 1.5);
  EXPECT_TRUE(m.voxels().empty());
  EXPECT_EQ(m.cleared(), 1u);
}
TEST(Memory, OccludedOutsideAndInvalidAreNotFree) {
  Memory m(.05, 100);
  m.observe({{0, 0, 2}}, 1.);
  for (int i = 0; i < 10; ++i)
    clear(m, image(1), 2. + i);
  EXPECT_EQ(m.voxels().begin()->second.visibility, Visibility::Occluded);
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation().x() = 5.;
  for (int i = 0; i < 10; ++i)
    clear(m, image(3), 12. + i, tf);
  EXPECT_EQ(m.voxels().begin()->second.visibility, Visibility::Outside);
  for (float invalid : {0.f, std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::infinity(), 5.f}) {
    for (int i = 0; i < 10; ++i)
      clear(m, image(invalid), 22. + i);
    ASSERT_EQ(m.voxels().size(), 1u);
    EXPECT_EQ(m.voxels().begin()->second.visibility, Visibility::Uncertain);
  }
}
TEST(Memory, FreeEvidenceMustBeConsecutive) {
  Memory m(.05, 100);
  m.observe({{0, 0, 2}}, 1.);
  for (int i = 0; i < 4; ++i)
    clear(m, image(3), 2. + i);
  clear(m, image(1), 6.);
  for (int i = 0; i < 4; ++i)
    clear(m, image(3), 7. + i);
  EXPECT_EQ(m.voxels().size(), 1u);
  clear(m, image(3), 11.);
  EXPECT_TRUE(m.voxels().empty());
}
TEST(Memory, ObservationWinsAndMemoryDoesNotExpire) {
  Memory m(.05, 100);
  m.observe({{0, 0, 2}}, 1.);
  m.observe({{0, 0, 2}}, 1000.);
  clear(m, image(3), 1000.);
  ASSERT_EQ(m.voxels().size(), 1u);
  EXPECT_EQ(m.voxels().begin()->second.free_frames, 0);
}
TEST(Memory, CapacityArchivesAndReloadsUnseenObstacles) {
  Memory m(.05, 1);
  EXPECT_TRUE(m.observe({{0, 0, 2}}, 1.));
  EXPECT_TRUE(m.observe({{1, 0, 2}}, 2.));
  ASSERT_EQ(m.voxels().size(), 1u);
  EXPECT_DOUBLE_EQ(m.voxels().begin()->second.point.x(), 0);
  EXPECT_EQ(m.archivedCount(), 1u);
  m.recenter({1, 0, 0}, .2);
  ASSERT_EQ(m.voxels().size(), 1u);
  EXPECT_DOUBLE_EQ(m.voxels().begin()->second.point.x(), 1);
  EXPECT_EQ(m.archivedCount(), 1u);
}
TEST(Memory, BoundedArchiveProjectionRetainsOccupancy) {
  Memory m(.05, 10);
  std::vector<Eigen::Vector3d> p;
  for (int x = -30; x < 30; ++x)
    for (int y = -5; y < 5; ++y)
      p.push_back({double(x), double(y), .08});
  m.observe(p, 1.);
  m.recenter({0, 0, 0}, 1.);
  EXPECT_LE(m.voxels().size(), 10u);
  EXPECT_EQ(m.voxels().size() + m.archivedCount(), p.size());
  auto projection = m.archiveProjection(8);
  EXPECT_LE(projection.second.size(), 8u);
  EXPECT_GT(projection.first, .1);
}
TEST(Memory, MovingOwnerRemovesOnlyItsOwnOldOccupancy) {
  Memory m(.05, 1);
  m.observe({{0, 0, 2}}, 1., 7);
  m.observe({{1, 0, 2}}, 1., 8);
  m.removeOwner(7);
  EXPECT_EQ(m.voxels().size() + m.archivedCount(), 1u);
  m.recenter({1, 0, 0}, 2.);
  EXPECT_EQ(m.voxels().begin()->second.owner, 8u);
}
TEST(Memory, MoreThanHundredThousandObservationsStayBounded) {
  Memory m(.05, 256);
  for (int batch = 0; batch < 21; ++batch) {
    std::vector<Eigen::Vector3d> points;
    for (int i = 0; i < 5000; ++i) {
      int k = batch * 5000 + i;
      points.push_back({(k % 1000) * .1, (k / 1000) * .1, .08});
    }
    m.observe(points, batch + 1.);
    m.recenter({50., batch * .5, 0}, 6.);
    EXPECT_LE(m.voxels().size(), 256u);
  }
  EXPECT_EQ(m.voxels().size() + m.archivedCount(), 105000u);
  auto projection = m.archiveProjection(256);
  EXPECT_LE(projection.second.size(), 256u);
  m.recenter({0, 0, 0}, 2.);
  EXPECT_GT(m.voxels().size(), 0u);
  EXPECT_LE(m.voxels().size(), 256u);
}
TEST(Memory, InvalidDepthInNeighbourhoodPreventsClearing) {
  auto d = image(3);
  d.at<float>(49, 49) = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(visibility({0, 0, 2}, d, k, .1, .12, 5.), Visibility::Uncertain);
}
TEST(Memory, DuplicateFreeImagesCannotClearAndResetStartsNewEpoch) {
  Memory m(.05, 1);
  m.observe({{0, 0, 2}, {1, 0, 2}}, 10.);
  ASSERT_EQ(m.voxels().size() + m.archivedCount(), 2u);
  for (int i = 0; i < 10; ++i)
    clear(m, image(3), 10.1);
  EXPECT_EQ(m.voxels().size() + m.archivedCount(), 2u);
  m.reset();
  EXPECT_TRUE(m.voxels().empty());
  EXPECT_EQ(m.archivedCount(), 0u);
  EXPECT_TRUE(m.archiveProjection().second.empty());
  EXPECT_EQ(m.cleared(), 0u);
  m.observe({{0, 0, 2}}, 1.);
  for (int i = 1; i <= 5; ++i)
    clear(m, image(3), 1. + i * .1);
  EXPECT_TRUE(m.voxels().empty());
  EXPECT_EQ(m.cleared(), 1u);
}

TEST(Memory, BatchArchiveReloadPreservesEveryBindingAndRepeatedFreeVotes) {
  Memory m(.05, 128);
  std::vector<Eigen::Vector3d> points;
  for (int x = -4; x <= 4; ++x)
    for (int y = -3; y <= 3; ++y)
      points.push_back({x * .1, y * .1, 2.});
  m.observe(points, 1., 123);
  m.recenter({20., 0., 0.}, .1);
  ASSERT_TRUE(m.voxels().empty());
  ASSERT_EQ(m.archivedCount(), points.size());
  clear(m, image(3), 1.1);
  clear(m, image(3), 1.2);
  m.recenter({0., 0., 0.}, 1.);
  ASSERT_EQ(m.voxels().size(), points.size());
  EXPECT_EQ(m.archivedCount(), 0u);
  for (const auto &[key, voxel] : m.voxels()) {
    (void)key;
    EXPECT_EQ(voxel.owner, 123u);
    EXPECT_DOUBLE_EQ(voxel.last_seen, 1.);
    EXPECT_EQ(voxel.free_frames, 2);
    EXPECT_DOUBLE_EQ(voxel.point.z(), 2.);
  }
  // Repeat the same prepared insert/update batches after a reset.
  m.reset();
  m.observe(points, 2., 456);
  m.recenter({20., 0., 0.}, .1);
  for (int i = 1; i <= 5; ++i)
    clear(m, image(3), 2. + i * .1);
  EXPECT_EQ(m.archivedCount(), 0u);
  EXPECT_EQ(m.cleared(), points.size());
}
