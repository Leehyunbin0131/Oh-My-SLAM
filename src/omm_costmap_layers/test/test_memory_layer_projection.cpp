#include "omm_costmap_layers/obstacle_memory_layer.hpp"
#include <gtest/gtest.h>
#include <rcl/time.h>
#include <limits>
#include <thread>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {
using Snapshot = omm_perception_msgs::msg::ObstacleMemory;
class MemoryLayerTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    node = std::make_shared<nav2_util::LifecycleNode>("memory_layer_fixture");
    publisher = std::make_shared<rclcpp::Node>("memory_layer_source");
    buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    pub = publisher->create_publisher<Snapshot>("/perception/memory", rclcpp::QoS(1).transient_local());
    group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    global = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    local = std::make_unique<nav2_costmap_2d::LayeredCostmap>("odom", true, false);
    global->resizeMap(100, 100, .05, -1., -2.);
    local->resizeMap(100, 100, .05, -1., -2.);
    map_layer = std::make_shared<omm_costmap_layers::ObstacleMemoryLayer>();
    odom_layer = std::make_shared<omm_costmap_layers::ObstacleMemoryLayer>();
    map_layer->initialize(global.get(), "global_memory", buffer.get(), node, group);
    odom_layer->initialize(local.get(), "local_memory", buffer.get(), node, group);
    global->addPlugin(map_layer);
    local->addPlugin(odom_layer);
    executor.add_node(node->get_node_base_interface());
    executor.add_node(publisher);
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (pub->get_subscription_count() < 2 && std::chrono::steady_clock::now() < end) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ASSERT_EQ(pub->get_subscription_count(), 2u);
    setTransform(0., 0.);
  }
  void TearDown() override {
    executor.remove_node(node->get_node_base_interface());
    executor.remove_node(publisher);
    map_layer.reset(); odom_layer.reset();
    global.reset(); local.reset();
    buffer.reset(); pub.reset(); node.reset(); publisher.reset();
  }
  void setTransform(double x, double yaw, double time_offset = 0., bool is_static = false) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = node->now() + rclcpp::Duration::from_seconds(time_offset);
    t.header.frame_id = "map";
    t.child_frame_id = "odom";
    t.transform.translation.x = x;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    t.transform.rotation = tf2::toMsg(q);
    // map->odom is dynamic in both SLAM and AMCL. StaticCache deliberately
    // survives Buffer::clear(), so a static fixture cannot test missing TF.
    ASSERT_TRUE(buffer->setTransform(t, "fixture", is_static));
  }
  Snapshot sample() {
    Snapshot s;
    s.header.stamp = node->now();
    s.observation_stamp = s.header.stamp;
    s.header.frame_id = "odom";
    s.sensor_healthy = s.geometry_healthy = s.memory_healthy = true;
    s.voxel_size = .05;
    s.archive_resolution = .10;
    geometry_msgs::msg::Point32 p;
    p.x = 1.; p.y = 0.; p.z = .1;
    s.points.push_back(p);
    p.x = 2.;
    s.archived_points.push_back(p);
    return s;
  }
  void send(const Snapshot &s, double robot_x = 0.) {
    pub->publish(s);
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(30);
    while (std::chrono::steady_clock::now() < end) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    global->updateMap(0, 0, 0);
    local->updateMap(robot_x, 0, 0);
  }
  unsigned char cost(nav2_costmap_2d::LayeredCostmap &map, double x, double y) {
    unsigned int mx, my;
    EXPECT_TRUE(map.getCostmap()->worldToMap(x, y, mx, my));
    return map.getCostmap()->getCost(mx, my);
  }
  std::shared_ptr<nav2_util::LifecycleNode> node;
  rclcpp::Node::SharedPtr publisher;
  rclcpp::Publisher<Snapshot>::SharedPtr pub;
  std::unique_ptr<tf2_ros::Buffer> buffer;
  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> global, local;
  std::shared_ptr<omm_costmap_layers::ObstacleMemoryLayer> map_layer, odom_layer;
};

TEST_F(MemoryLayerTest, ProjectionResetHealthAndOldBounds) {
  auto s = sample();
  send(s);
  ASSERT_TRUE(map_layer->isCurrent());
  ASSERT_TRUE(odom_layer->isCurrent());
  EXPECT_EQ(cost(*global, 1., 0.), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(*local, 1., 0.), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(*global, 2., 0.), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(*global, .925, -.075), nav2_costmap_2d::LETHAL_OBSTACLE);

  setTransform(.05, .05);
  send(sample());
  EXPECT_TRUE(map_layer->isCurrent());
  EXPECT_EQ(cost(*global, .925, -.075), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(*global, 1.05, .05), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(*local, .925, -.075), nav2_costmap_2d::LETHAL_OBSTACLE);
  map_layer->reset(); odom_layer->reset();
  EXPECT_FALSE(map_layer->isClearable());
  global->updateMap(0, 0, 0);
  EXPECT_EQ(cost(*global, 1.05, .05), nav2_costmap_2d::LETHAL_OBSTACLE);

  auto unhealthy = sample();
  unhealthy.geometry_healthy = false;
  unhealthy.points.clear(); unhealthy.archived_points.clear();
  send(unhealthy);
  EXPECT_FALSE(map_layer->isCurrent());
  EXPECT_EQ(cost(*global, 1.05, .05), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(*global, 2.05, .10), nav2_costmap_2d::LETHAL_OBSTACLE);
  auto stale = sample();
  stale.observation_stamp = node->now() - rclcpp::Duration::from_seconds(.5);
  stale.points.clear(); stale.archived_points.clear();
  send(stale);
  EXPECT_FALSE(map_layer->isCurrent());
  EXPECT_EQ(cost(*local, 1., 0.), nav2_costmap_2d::LETHAL_OBSTACLE);

  auto bad = sample();
  bad.points.front().x = std::numeric_limits<float>::quiet_NaN();
  send(bad);
  EXPECT_FALSE(map_layer->isCurrent());
  EXPECT_EQ(cost(*local, 1., 0.), nav2_costmap_2d::LETHAL_OBSTACLE);
  buffer->clear();
  ASSERT_FALSE(buffer->canTransform("map", "odom", tf2::TimePointZero));
  send(sample());
  EXPECT_FALSE(map_layer->isCurrent());
  EXPECT_EQ(cost(*global, 1.05, .05), nav2_costmap_2d::LETHAL_OBSTACLE);
  setTransform(.05, .05);
  auto clear = sample();
  clear.points.clear(); clear.archived_points.clear();
  send(clear);
  EXPECT_TRUE(map_layer->isCurrent());
  EXPECT_EQ(cost(*global, 1.05, .05), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(*local, 1., 0.), nav2_costmap_2d::FREE_SPACE);
}
TEST_F(MemoryLayerTest, BoundsCoverMarksWithoutDirtyingWholeGrid) {
  send(sample());
  double min_x = 1e30, min_y = 1e30, max_x = -1e30, max_y = -1e30;
  map_layer->updateBounds(0, 0, 0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_GT(min_x, .8);
  EXPECT_LT(max_x, 2.2);
  EXPECT_GT(min_y, -.2);
  EXPECT_LT(max_y, .2);
  EXPECT_LE(min_x, 1. - .085 - .025);
  EXPECT_GE(max_x, 2. + .06 + std::sqrt(.5) * .1 + .025);
}

TEST_F(MemoryLayerTest, RollingOriginMarksEnteringObstaclesAndClearsOldCells) {
  auto s = sample();
  s.archived_points.front().x = 4.;
  send(s);
  local->updateMap(3., 0, 0);
  EXPECT_EQ(cost(*local, 1., 0.), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(*local, 4., 0.), nav2_costmap_2d::LETHAL_OBSTACLE);
  auto clear = sample();
  clear.points.clear(); clear.archived_points.clear();
  send(clear, 3.);
  EXPECT_EQ(cost(*local, 1., 0.), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(*local, 4., 0.), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(MemoryLayerTest, ClockRewindClearsPreviousEpochThenAcceptsNewSnapshot) {
  auto clock = node->get_clock();
  ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 10000000000LL), RCL_RET_OK);
  buffer->clear();
  setTransform(0., 0.);
  send(sample());
  ASSERT_EQ(cost(*local, 1., 0.), nav2_costmap_2d::LETHAL_OBSTACLE);
  // Even a sub-second rewind establishes a new epoch.
  ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 9500000000LL), RCL_RET_OK);
  local->updateMap(0, 0, 0);
  global->updateMap(0, 0, 0);
  EXPECT_FALSE(odom_layer->isCurrent());
  EXPECT_EQ(cost(*local, 1., 0.), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(*global, 1., 0.), nav2_costmap_2d::FREE_SPACE);
  ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 10050000000LL), RCL_RET_OK);
  local->updateMap(0, 0, 0);
  global->updateMap(0, 0, 0);
  EXPECT_FALSE(odom_layer->isCurrent());
  EXPECT_FALSE(map_layer->isCurrent());
  EXPECT_EQ(cost(*local, 1., 0.), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(*global, 1., 0.), nav2_costmap_2d::FREE_SPACE);
  auto fresh = sample();
  fresh.points.front().x = 1.5;
  send(fresh);
  EXPECT_TRUE(odom_layer->isCurrent());
  EXPECT_EQ(cost(*local, 1.5, 0.), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(*local, 1., 0.), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(MemoryLayerTest, ProjectionRequiresFreshDynamicTransformAndRetainsLastMarks) {
  send(sample());
  ASSERT_TRUE(map_layer->isCurrent());
  buffer->clear();
  setTransform(.75, 0., -3.);
  send(sample());
  EXPECT_FALSE(map_layer->isCurrent());
  EXPECT_TRUE(odom_layer->isCurrent());
  EXPECT_EQ(cost(*global, 1., 0.), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(*global, 1.75, 0.), nav2_costmap_2d::FREE_SPACE);
  setTransform(.5, 0., .95);
  send(sample());
  EXPECT_TRUE(map_layer->isCurrent());
  EXPECT_EQ(cost(*global, 1.5, 0.), nav2_costmap_2d::LETHAL_OBSTACLE);
  setTransform(0., 0., 2.);
  send(sample());
  EXPECT_FALSE(map_layer->isCurrent());
  buffer->clear();
  setTransform(0., 0., -3., true);
  send(sample());
  EXPECT_TRUE(map_layer->isCurrent());
  EXPECT_EQ(cost(*global, 1., 0.), nav2_costmap_2d::LETHAL_OBSTACLE);
}
} // namespace
