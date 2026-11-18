#include "omm_perception/ground.hpp"
#include "omm_perception/tracker.hpp"
#include "omm_perception/self_mask.hpp"
#include <gtest/gtest.h>
using namespace omm_perception;
TEST(Ground, TiltAndSlopePreserveLowBox) {
  for (double pitch : {-.12, 0., .12}) {
    Eigen::Matrix3d tilt =
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
    std::vector<Eigen::Vector3d> points;
    for (int x = 4; x < 60; ++x)
      for (int y = -20; y < 20; ++y)
        points.push_back(tilt * Eigen::Vector3d(x * .05, y * .05, 0));
    for (int x = 26; x < 33; ++x)
      for (int y = -5; y < 6; ++y)
        points.push_back(tilt * Eigen::Vector3d(x * .05, y * .05, .08));
    auto plane = estimateGround(points, Eigen::Vector3d::UnitZ());
    ASSERT_TRUE(plane.valid);
    EXPECT_NEAR(plane.height(tilt * Eigen::Vector3d(2., 0, 0)), 0., .005);
    EXPECT_NEAR(plane.height(tilt * Eigen::Vector3d(1.5, 0, .08)), .08, .005);
  }
}
TEST(Ground, LowObstacleTopIsNotInventedAsFloor) {
  std::vector<Eigen::Vector3d> points;
  for (int x = 10; x < 60; ++x)
    for (int y = -20; y < 20; ++y)
      points.push_back({x * .05, y * .05, .08});
  EXPECT_FALSE(estimateGround(points, Eigen::Vector3d::UnitZ()).valid);
}
TEST(Ground, SmallVisibleFloorPatchNearWall) {
  std::vector<Eigen::Vector3d> points;
  for (int x = 0; x < 20; ++x)
    for (int y = -30; y < 30; ++y)
      points.push_back({.36 + x * .006, y * .01, 0.});
  for (int z = 1; z < 55; ++z)
    for (int y = -30; y < 30; ++y)
      points.push_back({.50, y * .01, z * .005});
  auto floor = estimateGround(points, Eigen::Vector3d::UnitZ());
  ASSERT_TRUE(floor.valid);
  EXPECT_NEAR(floor.height({.4, 0, 0}), 0., .003);
}
Cluster box(double x, double y) {
  Cluster c;
  c.center = {x, y, .08};
  c.extent = {.3, .4, .08};
  for (int i = -3; i <= 3; ++i)
    for (int j = -4; j <= 4; ++j)
      c.points.push_back({x + i * .05, y + j * .05, .08});
  return c;
}
TEST(Tracking, StationaryNoiseDoesNotCreateMotion) {
  Tracker t;
  for (int i = 0; i < 60; ++i)
    t.update({box(1.5 + .003 * std::sin(i), .003 * std::cos(i))}, i * .1 + 1.);
  ASSERT_EQ(t.tracks().size(), 1u);
  EXPECT_FALSE(t.tracks()[0].moving);
  EXPECT_TRUE(t.tracks()[0].stationary);
}
TEST(Tracking, MovingObjectDoesNotLeavePermanentTrail) {
  Tracker t;
  for (int i = 0; i < 40; ++i)
    t.update({box(1.5, i * .04)}, i * .1 + 1.);
  ASSERT_EQ(t.tracks().size(), 1u);
  EXPECT_TRUE(t.tracks()[0].moving);
  EXPECT_NEAR(t.tracks()[0].velocity.y(), .4, .02);
  EXPECT_FALSE(t.dynamicPoints(5.).empty());
  EXPECT_TRUE(t.dynamicPoints(6.).empty());
  t.update({}, 7.);
  EXPECT_TRUE(t.tracks().empty());
}
TEST(Tracking, ClippedWallWindowDoesNotPredictCorridorOccupancy) {
  // Integrated 011 observed wall fragments had z extent 0.259..0.283 m,
  // XY thickness <=0.004 m. Their visible centroid shifted along the wall
  // as the robot moved, although the wall normal coordinate stayed fixed.
  for (double wall_y : {-.4, .4}) {
    Tracker wall, low_object;
    for (int i = 0; i < 20; ++i) {
      Cluster c;
      c.center = {3. + .02 * i, wall_y, .1613};
      c.extent = {.1305, .0003, .2795};
      for (int x = -3; x <= 3; ++x)
        for (int z = -6; z <= 6; ++z)
          c.points.push_back(c.center + Eigen::Vector3d(
              x * c.extent.x() / 6., x * c.extent.y() / 6.,
              z * c.extent.z() / 12.));
      wall.update({c}, 1. + .1 * i);
      // The same tangential displacement of a genuinely low box must still
      // be classified as motion; no global prediction disable is acceptable.
      low_object.update({box(3. + .02 * i, wall_y)}, 1. + .1 * i);
    }
    ASSERT_EQ(wall.tracks().size(), 1u);
    EXPECT_FALSE(wall.tracks().front().moving);
    auto projected = wall.dynamicPoints(3.2);
    ASSERT_FALSE(projected.empty());
    for (const auto &p : projected)
      EXPECT_NEAR(p.y(), wall_y, .004);
    ASSERT_EQ(low_object.tracks().size(), 1u);
    EXPECT_TRUE(low_object.tracks().front().moving);
    EXPECT_FALSE(low_object.dynamicPoints(3.2).empty());
  }
}
TEST(Tracking, StoppedMoverCanBecomeStaticAndMoveAgain) {
  Tracker t;
  for (int i = 0; i < 30; ++i)
    t.update({box(1.5, i * .04)}, 1. + i * .1);
  for (int i = 0; i < 25; ++i)
    t.update({box(1.5, 1.16)}, 4. + i * .1);
  ASSERT_EQ(t.tracks().size(), 1u);
  EXPECT_TRUE(t.tracks()[0].stationary);
  EXPECT_FALSE(t.tracks()[0].moving);
  for (int i = 0; i < 20; ++i)
    t.update({box(1.5, 1.16 + i * .04)}, 6.5 + i * .1);
  ASSERT_EQ(t.tracks().size(), 1u);
  EXPECT_TRUE(t.tracks()[0].moving);
  EXPECT_FALSE(t.tracks()[0].stationary);
}
TEST(Geometry, SelfMaskFollowsCadAndKeepsAdjacentLowObstacles) {
  EXPECT_TRUE(insideRobot({-.14, 0, .147}));
  EXPECT_TRUE(insideRobot({0, .24, .068}));
  EXPECT_TRUE(insideRobot({0, -.24, .068}));
  EXPECT_TRUE(insideRobot({-.38, .155, .038}));
  EXPECT_TRUE(insideRobot({-.30775, -.225, .038}));
  EXPECT_TRUE(insideRobot({.031055, .03875, .2045}));
  EXPECT_FALSE(insideRobot({.12, 0, .08}));
  EXPECT_FALSE(insideRobot({-.42, .155, .08}));
  EXPECT_FALSE(insideRobot({0, .29, .08}));
  EXPECT_FALSE(insideRobot({.08, 0, .10}));
  EXPECT_FALSE(insideRobot({-.14, 0, .02}));
}
TEST(Tracking, SessionResetDropsDynamicAndStaticAssociations) {
  Tracker t;
  for (int i = 0; i < 40; ++i)
    t.update({box(1.5, i * .04)}, 10. + i * .1);
  ASSERT_FALSE(t.tracks().empty());
  t.reset();
  EXPECT_TRUE(t.tracks().empty());
  EXPECT_TRUE(t.dynamicPoints(1.).empty());
  t.update({box(1.5, 0)}, 1.);
  ASSERT_EQ(t.tracks().size(), 1u);
  EXPECT_EQ(t.tracks().front().id, 1u);
  EXPECT_EQ(t.tracks().front().observations, 1);
}
