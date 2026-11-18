#include "omm_perception/node.hpp"
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <cstring>
#include <functional>
#include <thread>
#include <omm_perception_msgs/msg/obstacle_memory.hpp>
#include <omm_perception_msgs/msg/memory_status.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace {
using Snapshot = omm_perception_msgs::msg::ObstacleMemory;
class SessionTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true),
                                 rclcpp::Parameter("max_voxels", 16)});
    perception = omm_perception::makeObstacleMemory(options);
    source = std::make_shared<rclcpp::Node>("perception_fixture", options);
    clock = source->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    info_pub = source->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/depth/camera_info", rclcpp::SensorDataQoS());
    imu_pub = source->create_publisher<sensor_msgs::msg::Imu>("/imu/data", rclcpp::SensorDataQoS());
    depth_pub = source->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_rect_raw", rclcpp::SensorDataQoS());
    tf_pub = source->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", rclcpp::QoS(1).transient_local());
    memory_sub = source->create_subscription<Snapshot>("/perception/memory", rclcpp::QoS(1).transient_local(),
        [this](Snapshot::ConstSharedPtr s) { snapshot = s; });
    status_sub = source->create_subscription<omm_perception_msgs::msg::MemoryStatus>(
        "/perception/memory_status", rclcpp::QoS(1).transient_local(),
        [this](omm_perception_msgs::msg::MemoryStatus::ConstSharedPtr s) { status = s; });
    preview_sub = source->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/perception/memory_preview", rclcpp::QoS(1).transient_local(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr s) { preview = s; });
    executor.add_node(perception);
    executor.add_node(source);
    ASSERT_TRUE(until([&] { return depth_pub->get_subscription_count() > 0 &&
        info_pub->get_subscription_count() > 0 && imu_pub->get_subscription_count() > 0 &&
        clock->get_subscription_count() >= 2 && tf_pub->get_subscription_count() > 0; }));
    setTime(10.);
    tf2_msgs::msg::TFMessage transforms;
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp(10.);
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";
    tf.transform.rotation.w = 1.;
    transforms.transforms.push_back(tf);
    tf.header.frame_id = "base_footprint";
    tf.child_frame_id = "test_imu";
    transforms.transforms.push_back(tf);
    tf.child_frame_id = "test_optical";
    tf.transform.translation.x = .06505;
    tf.transform.translation.z = .2045;
    Eigen::Matrix3d rotation;
    rotation << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    Eigen::Quaterniond q(rotation);
    tf.transform.rotation.w = q.w(); tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y(); tf.transform.rotation.z = q.z();
    transforms.transforms.push_back(tf);
    static_transforms = transforms;
    tf_pub->publish(transforms);
    pump(.08);
  }
  void TearDown() override {
    executor.remove_node(perception);
    executor.remove_node(source);
    perception.reset();
    source.reset();
  }
  bool until(const std::function<bool()> &condition, double seconds = 2.) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (!condition() && std::chrono::steady_clock::now() < end) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return condition();
  }
  void pump(double seconds) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    until([&] { return std::chrono::steady_clock::now() >= end; }, seconds + .05);
  }
  builtin_interfaces::msg::Time stamp(double value) {
    return rclcpp::Time(int64_t(std::llround(value * 1e9)));
  }
  void setTime(double t) {
    rosgraph_msgs::msg::Clock message;
    message.clock = stamp(t);
    clock->publish(message);
    ASSERT_TRUE(until([&] { return perception->now().nanoseconds() == rclcpp::Time(message.clock).nanoseconds() &&
                                  source->now().nanoseconds() == rclcpp::Time(message.clock).nanoseconds(); }));
    pump(.02);
  }
  void cameraInfo(double t) {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp(t); info.header.frame_id = "test_optical";
    info.width = 160; info.height = 120;
    info.k = {100., 0., 79.5, 0., 100., 59.5, 0., 0., 1.};
    info_pub->publish(info);
  }
  void imu(double t) {
    sensor_msgs::msg::Imu m;
    m.header.stamp = stamp(t); m.header.frame_id = "test_imu";
    m.orientation.w = 1.;
    imu_pub->publish(m);
  }
  sensor_msgs::msg::Image image(double t, bool with_box, bool millimeters) {
    sensor_msgs::msg::Image m;
    m.header.stamp = stamp(t); m.header.frame_id = "test_optical";
    m.width = 160; m.height = 120;
    m.encoding = millimeters ? "16UC1" : "32FC1";
    m.step = m.width * (millimeters ? 2 : 4);
    m.data.resize(m.step * m.height);
    Eigen::Vector3d origin(.06505, 0., .2045);
    for (unsigned int v = 0; v < m.height; ++v)
      for (unsigned int u = 0; u < m.width; ++u) {
        Eigen::Vector3d ray(1., -(double(u) - 79.5) / 100., -(double(v) - 59.5) / 100.);
        double distance = ray.z() < 0 ? -origin.z() / ray.z() : 10.;
        if (with_box) {
          double near = 0., far = 10.;
          const Eigen::Vector3d lo(1., -.15, 0.), hi(1.3, .15, .10);
          for (int axis = 0; axis < 3; ++axis) {
            if (std::abs(ray[axis]) < 1e-10) {
              if (origin[axis] < lo[axis] || origin[axis] > hi[axis]) far = -1.;
            } else {
              double a = (lo[axis] - origin[axis]) / ray[axis];
              double b = (hi[axis] - origin[axis]) / ray[axis];
              if (a > b) std::swap(a, b);
              near = std::max(near, a); far = std::min(far, b);
            }
          }
          if (near <= far && near > 0) distance = std::min(distance, near);
        }
        if (millimeters) {
          uint16_t value = uint16_t(std::lround(distance * 1000.));
          std::memcpy(&m.data[v * m.step + u * 2], &value, sizeof(value));
        } else {
          float value = distance;
          std::memcpy(&m.data[v * m.step + u * 4], &value, sizeof(value));
        }
      }
    return m;
  }
  void observation(double t, bool box, bool mm, bool send_imu = true) {
    setTime(t);
    cameraInfo(t);
    if (send_imu) imu(t);
    pump(.02);
    depth_pub->publish(image(t, box, mm));
  }
  rclcpp::Node::SharedPtr perception, source;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub;
  rclcpp::Subscription<Snapshot>::SharedPtr memory_sub;
  Snapshot::ConstSharedPtr snapshot;
  rclcpp::Subscription<omm_perception_msgs::msg::MemoryStatus>::SharedPtr status_sub;
  omm_perception_msgs::msg::MemoryStatus::ConstSharedPtr status;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr preview_sub;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr preview;
  tf2_msgs::msg::TFMessage static_transforms;
  rclcpp::executors::SingleThreadedExecutor executor;
};

TEST_F(SessionTest, CompactStatusAndPreviewMatchFullSnapshot) {
  observation(10., true, false);
  ASSERT_TRUE(until([&] { return snapshot && status && preview &&
      snapshot->sensor_healthy && status->sensor_healthy &&
      preview->width * preview->height > 0; }));
  EXPECT_EQ(status->header, snapshot->header);
  EXPECT_EQ(status->observation_stamp, snapshot->observation_stamp);
  EXPECT_EQ(status->geometry_healthy, snapshot->geometry_healthy);
  EXPECT_EQ(status->memory_healthy, snapshot->memory_healthy);
  EXPECT_EQ(status->status, snapshot->status);
  EXPECT_EQ(status->voxel_size, snapshot->voxel_size);
  EXPECT_EQ(status->active_points, snapshot->points.size());
  EXPECT_EQ(status->archived_points, snapshot->archived_points.size());
  EXPECT_EQ(status->archived_voxels, snapshot->archived_voxels);
  EXPECT_GT(status->archived_points, 0u);
  const size_t total = snapshot->points.size() + snapshot->archived_points.size();
  const size_t step = std::max(size_t(1), (total + 3999) / 4000);
  EXPECT_EQ(preview->width * preview->height, (total + step - 1) / step);
  EXPECT_LE(preview->width * preview->height, 4000u);
  EXPECT_EQ(preview->header, snapshot->header);
  auto invalid = image(10.1, false, false);
  std::fill(invalid.data.begin(), invalid.data.end(), 0);
  setTime(10.1); imu(10.1); pump(.01); depth_pub->publish(invalid);
  ASSERT_TRUE(until([&] { return snapshot && status && !snapshot->sensor_healthy &&
      !status->sensor_healthy && status->header.stamp == stamp(10.1); }));
  EXPECT_EQ(status->observation_stamp, stamp(10.));
  EXPECT_EQ(status->status, snapshot->status);
  setTime(1.);
  ASSERT_TRUE(until([&] { return status->header.stamp == stamp(1.) &&
      status->observation_stamp == stamp(0.) && preview->header.stamp == stamp(1.); }));
  EXPECT_FALSE(status->sensor_healthy);
  EXPECT_FALSE(status->geometry_healthy);
  EXPECT_EQ(status->active_points, 0u);
  EXPECT_EQ(status->archived_points, 0u);
  EXPECT_EQ(preview->width * preview->height, 0u);
}

TEST_F(SessionTest, EncodingsFreshnessAndClockReset) {
  observation(10., true, false);
  ASSERT_TRUE(until([&] { return snapshot && snapshot->sensor_healthy && snapshot->geometry_healthy &&
      snapshot->memory_healthy && !snapshot->points.empty(); }));
  EXPECT_EQ(snapshot->header.frame_id, "odom");
  EXPECT_EQ(snapshot->observation_stamp, stamp(10.));
  EXPECT_GT(snapshot->archived_voxels, 0u);
  bool box_seen = false;
  for (const auto &p : snapshot->points)
    box_seen |= p.x >= .95 && p.x <= 1.35 && p.z >= .02 && p.z <= .11;
  EXPECT_TRUE(box_seen);
  const auto first_count = snapshot->points.size() + snapshot->archived_voxels;
  observation(10.1, true, true);
  ASSERT_TRUE(until([&] { return snapshot && snapshot->sensor_healthy && snapshot->geometry_healthy &&
      snapshot->observation_stamp == stamp(10.1); }));
  EXPECT_GT(snapshot->points.size() + snapshot->archived_voxels, first_count / 2);
  auto invalid = image(10.2, false, false);
  std::fill(invalid.data.begin(), invalid.data.end(), 0);
  setTime(10.2); imu(10.2); pump(.01); depth_pub->publish(invalid);
  ASSERT_TRUE(until([&] { return snapshot && !snapshot->sensor_healthy; }));
  EXPECT_EQ(snapshot->observation_stamp, stamp(10.1));
  EXPECT_GT(snapshot->points.size() + snapshot->archived_voxels, 0u);
  // Leave an image waiting for a matching IMU when the simulator clock rewinds.
  observation(10.3, false, false, false);
  pump(.03);
  setTime(1.);
  ASSERT_TRUE(until([&] { return snapshot && snapshot->header.stamp == stamp(1.) &&
      snapshot->observation_stamp == stamp(0.); }));
  EXPECT_FALSE(snapshot->sensor_healthy);
  EXPECT_FALSE(snapshot->geometry_healthy);
  EXPECT_TRUE(snapshot->points.empty());
  EXPECT_EQ(snapshot->archived_voxels, 0u);
  EXPECT_EQ(snapshot->dynamic_tracks, 0u);
  imu(10.3); pump(.03);
  EXPECT_EQ(snapshot->observation_stamp, stamp(0.));
  // TF2 also clears its cache on clock reset. Re-establish this session's TF.
  tf_pub->publish(static_transforms); pump(.04);
  observation(1.1, false, false);
  ASSERT_TRUE(until([&] { return snapshot && snapshot->sensor_healthy && snapshot->geometry_healthy &&
      snapshot->observation_stamp == stamp(1.1); }));
  EXPECT_TRUE(snapshot->memory_healthy);
  EXPECT_TRUE(snapshot->points.empty());
  EXPECT_EQ(snapshot->archived_voxels, 0u);
}

TEST_F(SessionTest, FutureSensorsWaitForClockWithoutRenewingReceipt) {
  observation(10., true, false);
  ASSERT_TRUE(until([&] { return snapshot && snapshot->sensor_healthy &&
      snapshot->observation_stamp == stamp(10.); }));
  setTime(10.02);
  cameraInfo(10.1);
  imu(10.1);
  depth_pub->publish(image(10.1, true, false));
  pump(.04);
  // A camera/IMU pair can precede its matching /clock by more than 50ms.
  // It must neither change the original observation early nor be discarded.
  EXPECT_EQ(snapshot->observation_stamp, stamp(10.));
  setTime(10.12);
  ASSERT_TRUE(until([&] { return snapshot && snapshot->sensor_healthy &&
      snapshot->geometry_healthy && snapshot->observation_stamp == stamp(10.1); }));

  // Waiting never renews receipt time. Catch-up after the 300ms wall deadline
  // cannot resurrect a queued observation or change its stamp to "now".
  cameraInfo(10.2);
  imu(10.2);
  depth_pub->publish(image(10.2, true, false));
  pump(.36);
  setTime(10.22);
  pump(.12);
  EXPECT_EQ(snapshot->observation_stamp, stamp(10.1));
  EXPECT_FALSE(snapshot->sensor_healthy);
  observation(10.3, true, false);
  ASSERT_TRUE(until([&] { return snapshot && snapshot->sensor_healthy &&
      snapshot->observation_stamp == stamp(10.3); }));
}
} // namespace
