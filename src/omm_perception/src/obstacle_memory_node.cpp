#include "omm_perception/ground.hpp"
#include "omm_perception/memory.hpp"
#include "omm_perception/node.hpp"
#include "omm_perception/self_mask.hpp"
#include "omm_perception/tracker.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
#include <deque>
#include <exception>
#include <limits>
#include <omm_perception_msgs/msg/obstacle_memory.hpp>
#include <omm_perception_msgs/msg/memory_status.hpp>
#include <omm_perception_msgs/msg/perception_health.hpp>
#include <optional>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>

namespace omm_perception {
Eigen::Isometry3d eigenTransform(const geometry_msgs::msg::Transform &t) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() =
      Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z)
          .normalized()
          .toRotationMatrix();
  result.translation() =
      Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
  return result;
}

class ObstacleMemory : public rclcpp::Node {
public:
  explicit ObstacleMemory(const rclcpp::NodeOptions &options)
      : Node("obstacle_memory", options),
        voxel_size_(declareReadOnlyVoxelSize()),
        memory_(voxel_size_, declare_parameter("max_voxels", 100000)),
        buffer_(get_clock()), listener_(buffer_) {
    memory_frame_ = declare_parameter("memory_frame", std::string("odom"));
    base_frame_ =
        declare_parameter("base_frame", std::string("base_footprint"));
    min_height_ = declare_parameter("min_height", 0.02);
    max_height_ = declare_parameter("max_height", 0.30);
    min_depth_ = declare_parameter("min_depth", .12);
    max_depth_ = declare_parameter("max_depth", 5.0);
    timeout_ = declare_parameter("sensor_timeout", .3);
    margin_ = declare_parameter("clear_margin", .10);
    free_frames_ = declare_parameter("free_frames", 5);
    stride_ = declare_parameter("pixel_stride", 2);
    active_radius_ = declare_parameter("active_radius", 6.);
    archive_cells_ = declare_parameter("max_archive_cells", 4096);
    max_slope_ = declare_parameter("max_ground_slope", .35);
    require_imu_ = declare_parameter("require_imu", true);
    if (stride_ < 1 || free_frames_ < 5 || min_height_ >= max_height_ ||
        min_depth_ >= max_depth_ || !std::isfinite(timeout_) || timeout_ <= 0 || active_radius_ < 1. ||
        archive_cells_ < 4 || max_slope_ <= 0 || max_slope_ > .7)
      throw std::invalid_argument("Invalid perception parameters");
    rcl_jump_threshold_t threshold{};
    threshold.on_clock_change = true;
    threshold.min_backward.nanoseconds = -1;
    jump_handler_ = get_clock()->create_jump_callback(
        nullptr, [this](const rcl_time_jump_t &) { reset_pending_ = true; }, threshold);
    raw_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/camera/points", rclcpp::SensorDataQoS());
    obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/obstacles", rclcpp::SensorDataQoS());
    memory_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/memory_cloud", rclcpp::QoS(1).transient_local());
    memory_pub_ = create_publisher<omm_perception_msgs::msg::ObstacleMemory>(
        "/perception/memory", rclcpp::QoS(1).transient_local());
    memory_status_pub_ = create_publisher<omm_perception_msgs::msg::MemoryStatus>(
        "/perception/memory_status", rclcpp::QoS(1).transient_local());
    memory_preview_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/memory_preview", rclcpp::QoS(1).transient_local());
    health_pub_ = create_publisher<omm_perception_msgs::msg::PerceptionHealth>(
        "/perception/health", rclcpp::QoS(1).transient_local());
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/depth/camera_info", rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
          checkSession();
          info_ = m;
        });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
          checkSession();
          const auto stamp = rclcpp::Time(m->header.stamp).seconds();
          const auto now = get_clock()->now().seconds();
          if (stamp <= 0 || now - stamp > timeout_)
            return;
          // Sensor and /clock streams can arrive out of order. Cache future
          // IMU samples, but alignedImu never uses them before clock catch-up.
          imu_history_.push_back(m);
          if (imu_history_.size() > 100)
            imu_history_.pop_front();
          retryPendingDepth();
        });
    // Simulation requires reliable depth delivery; retain sensor QoS by default
    // for external cameras whose offered reliability is configured separately.
    const auto depth_qos = declare_parameter("depth_reliable", false)
                               ? rclcpp::QoS(5).reliable()
                               : rclcpp::SensorDataQoS().keep_last(1);
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth/image_rect_raw", depth_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
          checkSession();
          const auto stamp = rclcpp::Time(m->header.stamp);
          auto pos = std::lower_bound(pending_depth_.begin(), pending_depth_.end(), stamp,
              [](const auto &entry, const auto &t) {return rclcpp::Time(entry.first->header.stamp) < t;});
          if (pos != pending_depth_.end() && pos->first->header.stamp == m->header.stamp)
            return;
          pending_depth_.emplace(pos, m, std::chrono::steady_clock::now());
          if (pending_depth_.size() > 8) {
            pending_depth_.pop_front();
            memory_.interruptFreeEvidence();
          }
          retryPendingDepth();
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               [this] { publishMemory(); });
  }

private:
  using SteadyClock = std::chrono::steady_clock;
  using ReceiptDeadline = std::chrono::time_point<SteadyClock, std::chrono::duration<double>>;

  double declareReadOnlyVoxelSize() {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter("voxel_size", .05, descriptor);
  }

  bool observationExpired(ReceiptDeadline deadline, double now) {
    return SteadyClock::now() >= deadline || reset_pending_.load() ||
           now < last_clock_ || !rclcpp::ok(get_node_base_interface()->get_context());
  }

  std::optional<geometry_msgs::msg::TransformStamped> lookupObservationTransform(
      const std::string &target, const std::string &source, const rclcpp::Time &stamp,
      const rclcpp::Duration &budget, ReceiptDeadline deadline) {
    const auto start = get_clock()->now();
    ReceiptDeadline next_attempt = SteadyClock::now();
    bool attempted = false;
    while (true) {
      const auto now = get_clock()->now();
      if (observationExpired(deadline, now.seconds()))
        return std::nullopt;
      // TF can arrive while this thread sleeps past the ROS budget. With the
      // receipt, shutdown and session checks above still passing, look once
      // more without waiting before reporting this leg's failure.
      const bool final_attempt = attempted && now - start >= budget;
      // The timed TF2 API sleeps on the ROS clock, which can pause in simulation.
      // The listener can receive TF while this callback waits on the steady clock.
      if (!final_attempt && SteadyClock::now() < next_attempt) {
        std::this_thread::sleep_until(next_attempt);
        continue;
      }
      try {
        return buffer_.lookupTransform(target, source, stamp);
      } catch (const tf2::TransformException &) {
        if (final_attempt)
          throw;
      }
      attempted = true;
      const ReceiptDeadline retry = SteadyClock::now() + std::chrono::milliseconds(10);
      next_attempt = std::min(retry, deadline);
    }
  }

  // Returns the clock sample this check validated for the current session.
  rclcpp::Time checkSession() {
    const auto sampled = get_clock()->now();
    const double now = sampled.seconds();
    if (reset_pending_.exchange(false) || now < last_clock_) {
      pending_depth_.clear();
      info_.reset();
      imu_history_.clear();
      tracker_.reset();
      last_floor_ = GroundPlane();
      isolated_points_.clear();
      last_stamp_ = -1.;
      last_observation_ = builtin_interfaces::msg::Time();
      healthy_ = false;
      geometry_healthy_ = false;
      memory_healthy_ = false;
      status_ = "new_clock_session_waiting_for_observations";
      try {
        memory_.reset();
        memory_failed_ = false;
        memory_healthy_ = true;
      } catch (const std::exception &e) {
        memory_failed_ = true;
        status_ = std::string("memory_reset_error: ") + e.what();
      }
    }
    last_clock_ = now;
    return sampled;
  }
  sensor_msgs::msg::Imu::ConstSharedPtr alignedImu(const rclcpp::Time &image_stamp) {
    const double stamp = image_stamp.seconds();
    sensor_msgs::msg::Imu::ConstSharedPtr before, after;
    double tb = -1e30, ta = 1e30;
    const double now = get_clock()->now().seconds();
    for (const auto &m : imu_history_) {
      double t = rclcpp::Time(m->header.stamp).seconds();
      if (t > now)
        continue;
      if (t <= stamp + 1e-6 && t > tb) {
        tb = t;
        before = m;
      }
      if (t >= stamp - 1e-6 && t < ta) {
        ta = t;
        after = m;
      }
    }
    if (!before || !after || stamp - tb > .05 || ta - stamp > .05 ||
        before->header.frame_id != after->header.frame_id)
      return {};
    const auto &a = before->orientation;
    const auto &b = after->orientation;
    Eigen::Quaterniond qa(a.w, a.x, a.y, a.z), qb(b.w, b.x, b.y, b.z);
    if (before->orientation_covariance[0] < 0 ||
        after->orientation_covariance[0] < 0 || !qa.coeffs().allFinite() ||
        !qb.coeffs().allFinite() || qa.norm() < .5 || qb.norm() < .5)
      return {};
    auto result = std::make_shared<sensor_msgs::msg::Imu>(*before);
    double alpha =
        ta - tb > 1e-6 ? std::clamp((stamp - tb) / (ta - tb), 0., 1.) : 0.;
    auto q = qa.normalized().slerp(alpha, qb.normalized());
    result->orientation.w = q.w();
    result->orientation.x = q.x();
    result->orientation.y = q.y();
    result->orientation.z = q.z();
    // Seconds lose nanoseconds at epoch times; keep the exact image stamp.
    result->header.stamp = image_stamp;
    return result;
  }
  sensor_msgs::msg::PointCloud2
  cloud(const std::vector<Eigen::Vector3d> &points,
        const std_msgs::msg::Header &header) {
    sensor_msgs::msg::PointCloud2 result;
    result.header = header;
    sensor_msgs::PointCloud2Modifier modifier(result);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    result.is_dense = true;
    sensor_msgs::PointCloud2Iterator<float> x(result, "x"), y(result, "y"),
        z(result, "z");
    for (const auto &p : points) {
      *x = p.x();
      *y = p.y();
      *z = p.z();
      ++x;
      ++y;
      ++z;
    }
    return result;
  }
  void retryPendingDepth() {
    while (!pending_depth_.empty()) {
      auto entry = pending_depth_.front();
      if (!depth(entry.first, entry.second))
        break;
      pending_depth_.pop_front();
    }
  }
  // A false result keeps the frame in timestamp order until alignment or expiry.
  bool depth(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
             std::chrono::steady_clock::time_point received) {
    const auto deadline = received + std::chrono::duration<double>(timeout_);
    const rclcpp::Time image_stamp(msg->header.stamp);
    const double stamp = image_stamp.seconds(), now = get_clock()->now().seconds();
    // Invalid, missing, duplicate, future or stale frames never provide
    // clearing evidence.
    bool previous_health = healthy_;
    healthy_ = false;
    status_ = "invalid_or_stale_observation";
    if (memory_failed_ || !info_ || stamp <= 0 || stamp <= last_stamp_ || now - stamp > timeout_ ||
        observationExpired(deadline, now) ||
        info_->header.frame_id != msg->header.frame_id ||
        msg->header.frame_id.empty() ||
        info_->width != msg->width || info_->height != msg->height ||
        msg->width < 8 || msg->height < 8 || !std::isfinite(info_->k[0]) ||
        !std::isfinite(info_->k[4]) || !std::isfinite(info_->k[2]) ||
        !std::isfinite(info_->k[5]) || info_->k[0] <= 0 || info_->k[4] <= 0) {
      if (stamp > last_stamp_)
        memory_.interruptFreeEvidence();
      return true;
    }
    if (stamp > now) {
      healthy_ = previous_health;
      status_ = "waiting_for_clock";
      return false;
    }
    auto imu = alignedImu(image_stamp);
    if (require_imu_ && !imu) {
      healthy_ = previous_health;
      status_ = "waiting_for_synchronized_imu";
      return false;
    }
    bool managing_memory = false;
    const auto reject_stale = [this] {
      memory_.interruptFreeEvidence();
      return true;
    };
    try {
      cv::Mat depth_image;
      if (msg->encoding == "32FC1")
        depth_image = cv_bridge::toCvCopy(msg, "32FC1")->image;
      else if (msg->encoding == "16UC1")
        cv_bridge::toCvCopy(msg, "16UC1")
            ->image.convertTo(depth_image, CV_32FC1, .001);
      else
        return reject_stale();
      const auto camera_budget = rclcpp::Duration::from_seconds(.08);
      const auto memory_tf = lookupObservationTransform(
          memory_frame_, msg->header.frame_id, image_stamp, camera_budget, deadline);
      if (!memory_tf)
        return reject_stale();
      const auto base_tf = lookupObservationTransform(
          base_frame_, msg->header.frame_id, image_stamp, camera_budget, deadline);
      if (!base_tf)
        return reject_stale();
      auto memory_from_camera = eigenTransform(memory_tf->transform);
      const auto base_from_camera = eigenTransform(base_tf->transform);
      auto memory_from_base = memory_from_camera * base_from_camera.inverse();
      Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
      if (imu) {
        const auto &q = imu->orientation;
        Eigen::Quaterniond world_from_imu(q.w, q.x, q.y, q.z);
        if (world_from_imu.coeffs().allFinite() && world_from_imu.norm() > .5) {
          // The IMU orientation is interpolated to the exact image stamp.
          const auto imu_tf = lookupObservationTransform(
              base_frame_, imu->header.frame_id, image_stamp,
              rclcpp::Duration::from_seconds(.03), deadline);
          if (!imu_tf)
            return reject_stale();
          const auto base_from_imu = eigenTransform(imu_tf->transform);
          up = base_from_imu.linear() *
               (world_from_imu.normalized().conjugate() *
                Eigen::Vector3d::UnitZ());
        }
      }
      const double after_tf = get_clock()->now().seconds();
      if (observationExpired(deadline, after_tf) || stamp > after_tf || after_tf - stamp > timeout_)
        return reject_stale();
      // Correct only the tilt missing from the supplied odom/TF chain. When TF
      // already includes roll/pitch this rotation is identity, not double
      // compensation.
      Eigen::Isometry3d tilt = Eigen::Isometry3d::Identity();
      tilt.linear() = Eigen::Quaterniond::FromTwoVectors(
                          up, memory_from_base.linear().transpose() *
                                  Eigen::Vector3d::UnitZ())
                          .toRotationMatrix();
      memory_from_camera = memory_from_base * tilt * base_from_camera;
      Intrinsics k{info_->k[0], info_->k[4], info_->k[2], info_->k[5]};
      std::vector<Eigen::Vector3d> raw, base_points, obstacles, observed;
      const bool want_raw = raw_pub_->get_subscription_count() > 0;
      int valid = 0, total = 0;
      for (int v = 0; v < depth_image.rows; v += stride_)
        for (int u = 0; u < depth_image.cols; u += stride_) {
          ++total;
          const float z = depth_image.at<float>(v, u);
          if (!std::isfinite(z) || z <= min_depth_ || z >= max_depth_)
            continue;
          ++valid;
          Eigen::Vector3d p((u - k.cx) * z / k.fx, (v - k.cy) * z / k.fy, z);
          if (want_raw)
            raw.push_back(p);
          base_points.push_back(base_from_camera * p);
        }
      // A dead image must not turn the entire remembered scene into free space.
      if (valid < 100 || valid < total * .05) {
        memory_.interruptFreeEvidence();
        return true;
      }
      healthy_ = true;
      const bool recent_floor = geometry_healthy_ && stamp - last_stamp_ < timeout_;
      last_stamp_ = stamp;
      last_observation_ = msg->header.stamp;
      auto ground = estimateGround(base_points, up, max_slope_, recent_floor ? &last_floor_ : nullptr);
      geometry_healthy_ = ground.valid;
      if (!ground.valid) {
        memory_.interruptFreeEvidence();
        status_ = "ground_not_established";
        return true;
      }
      last_floor_ = ground;
      for (const auto &b : base_points) {
        double height = ground.height(b);
        if (height < min_height_ || height > max_height_)
          continue;
        if (insideRobot(b))
          continue;
        obstacles.push_back(b);
        observed.push_back(memory_from_base * tilt * b);
      }
      managing_memory = true;
      auto clusters = clusterObstacles(observed);
      tracker_.update(clusters, stamp);
      isolated_points_.clear();
      for (const auto &c : clusters) {
        if (c.extent.head<2>().maxCoeff() > 1.6) {
          std::vector<Eigen::Vector3d> background;
          for (const auto &p : c.points)
            if (!tracker_.inMovingRegion(p, stamp))
              background.push_back(p);
          memory_.observe(background, stamp);
        } else if (c.points.size() < 3)
          isolated_points_.insert(isolated_points_.end(), c.points.begin(),
                                  c.points.end());
      }
      for (const auto &track : tracker_.tracks()) {
        if (track.moving)
          memory_.removeOwner(track.id);
        else if ((track.stationary || track.observations == 1) &&
                 track.last_seen == stamp)
          memory_.observe(track.cluster.points, stamp, track.id);
      }
      memory_.recenter(memory_from_base.translation(), active_radius_);
      memory_.clearVisible(depth_image, k, memory_from_camera.inverse(), stamp,
                           margin_, free_frames_, min_depth_, max_depth_, timeout_);
      memory_healthy_ = true;
      status_ = "ok";
      if (want_raw)
        raw_pub_->publish(cloud(raw, msg->header));
      auto header = msg->header;
      header.frame_id = base_frame_;
      // Keep the original sensor stamp so consumers can detect delayed
      // data.
      obstacle_pub_->publish(cloud(obstacles, header));
    } catch (const std::exception &e) {
      memory_.interruptFreeEvidence();
      if (managing_memory) {
        memory_failed_ = true;
        memory_healthy_ = false;
        status_ = std::string("memory_processing_error: ") + e.what();
      } else {
        healthy_ = false;
        status_ = e.what();
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Camera observation rejected: %s", e.what());
    }
    return true;
  }
  void publishMemory() {
    checkSession();
    // Retry on clock progress even when no new camera or IMU sample arrives.
    // The original receipt deadline is preserved across every retry.
    retryPendingDepth();
    // /clock can rewind while that retry waits for TF. Reset before this
    // snapshot so previous-session memory never carries a new-session stamp;
    // the snapshot uses the clock sample this check validated.
    const rclcpp::Time now = checkSession();
    omm_perception_msgs::msg::ObstacleMemory msg;
    msg.header.stamp = now;
    msg.header.frame_id = memory_frame_;
    msg.observation_stamp = last_observation_;
    const double age = now.seconds() - last_stamp_;
    msg.sensor_healthy = healthy_ && age >= 0 && age < timeout_;
    msg.geometry_healthy = geometry_healthy_;
    msg.memory_healthy = memory_healthy_;
    msg.status = status_;
    msg.voxel_size = voxel_size_;
    msg.cleared_voxels_total = memory_.cleared();
    std::vector<Eigen::Vector3d> points;
    points.reserve(memory_.voxels().size());
    for (const auto &[key, voxel] : memory_.voxels()) {
      (void)key;
      geometry_msgs::msg::Point32 point;
      point.x = voxel.point.x();
      point.y = voxel.point.y();
      point.z = voxel.point.z();
      msg.points.push_back(point);
      points.push_back(voxel.point);
      switch (voxel.visibility) {
      case Visibility::Visible:
        ++msg.visible_voxels;
        break;
      case Visibility::Occluded:
        ++msg.occluded_voxels;
        break;
      case Visibility::Outside:
        ++msg.outside_fov_voxels;
        break;
      default:
        ++msg.uncertain_voxels;
        break;
      }
    }
    auto dynamic = tracker_.dynamicPoints(now.seconds());
    if (age >= 0 && age < .3)
      dynamic.insert(dynamic.end(), isolated_points_.begin(),
                     isolated_points_.end());
    for (const auto &p : dynamic) {
      geometry_msgs::msg::Point32 v;
      v.x = p.x();
      v.y = p.y();
      v.z = p.z();
      msg.points.push_back(v);
      points.push_back(p);
    }
    for (const auto &t : tracker_.tracks())
      if (t.moving)
        ++msg.dynamic_tracks;
    try {
      auto archived = memory_.archiveProjection(archive_cells_);
      msg.archive_resolution = archived.first;
      msg.archived_voxels = memory_.archivedCount();
      for (const auto &p : archived.second) {
        geometry_msgs::msg::Point32 v;
        v.x = p.x();
        v.y = p.y();
        v.z = p.z();
        msg.archived_points.push_back(v);
      }
    } catch (const std::exception &e) {
      memory_failed_ = true;
      memory_healthy_ = false;
      msg.memory_healthy = false;
      msg.status = std::string("memory_archive_error: ") + e.what();
    }
    // Use the final snapshot flags, including any archive error above.
    omm_perception_msgs::msg::PerceptionHealth health;
    health.stamp = msg.header.stamp;
    health.observation_stamp = msg.observation_stamp;
    health.sensor_healthy = msg.sensor_healthy;
    health.geometry_healthy = msg.geometry_healthy;
    health.memory_healthy = msg.memory_healthy;
    health_pub_->publish(health);
    // Control and UI need metadata, not thousands of Python Point32 objects.
    // Copy only after archive errors have updated the completed snapshot.
    omm_perception_msgs::msg::MemoryStatus summary;
    summary.header = msg.header;
    summary.observation_stamp = msg.observation_stamp;
    summary.sensor_healthy = msg.sensor_healthy;
    summary.geometry_healthy = msg.geometry_healthy;
    summary.memory_healthy = msg.memory_healthy;
    summary.status = msg.status;
    summary.voxel_size = msg.voxel_size;
    summary.archive_resolution = msg.archive_resolution;
    summary.active_points = msg.points.size();
    summary.archived_points = msg.archived_points.size();
    summary.archived_voxels = msg.archived_voxels;
    summary.dynamic_tracks = msg.dynamic_tracks;
    summary.visible_voxels = msg.visible_voxels;
    summary.occluded_voxels = msg.occluded_voxels;
    summary.outside_fov_voxels = msg.outside_fov_voxels;
    summary.uncertain_voxels = msg.uncertain_voxels;
    summary.cleared_voxels_total = msg.cleared_voxels_total;
    memory_status_pub_->publish(summary);
    memory_pub_->publish(msg);
    if (memory_cloud_pub_->get_subscription_count() > 0)
      memory_cloud_pub_->publish(cloud(points, msg.header));
    if (memory_preview_pub_->get_subscription_count() > 0) {
      // Display-only sample includes archived geometry. Full navigation data
      // remains on /perception/memory without downsampling.
      const size_t total = msg.points.size() + msg.archived_points.size();
      const size_t step = std::max(size_t(1), (total + 3999) / 4000);
      std::vector<Eigen::Vector3d> preview;
      preview.reserve(std::min(total, size_t(4000)));
      for (size_t i = 0; i < total; i += step) {
        const auto &p = i < msg.points.size() ? msg.points[i] :
            msg.archived_points[i - msg.points.size()];
        preview.emplace_back(p.x, p.y, p.z);
      }
      memory_preview_pub_->publish(cloud(preview, msg.header));
    }
  }
  GroundPlane last_floor_;
  const double voxel_size_;
  Memory memory_;
  Tracker tracker_;
  std::vector<Eigen::Vector3d> isolated_points_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::string memory_frame_, base_frame_;
  double min_height_, max_height_, min_depth_, max_depth_, timeout_, margin_,
      last_stamp_ = -1.;
  int free_frames_, stride_;
  int archive_cells_;
  double active_radius_, max_slope_;
  bool healthy_ = false;
  bool geometry_healthy_ = false, memory_healthy_ = true;
  bool memory_failed_ = false;
  bool require_imu_ = true;
  std::string status_ = "waiting_for_camera";
  builtin_interfaces::msg::Time last_observation_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_history_;
  std::deque<std::pair<sensor_msgs::msg::Image::ConstSharedPtr,
                       std::chrono::steady_clock::time_point>> pending_depth_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pub_,
      obstacle_pub_, memory_cloud_pub_;
  rclcpp::Publisher<omm_perception_msgs::msg::ObstacleMemory>::SharedPtr
      memory_pub_;
  rclcpp::Publisher<omm_perception_msgs::msg::MemoryStatus>::SharedPtr memory_status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr memory_preview_pub_;
  rclcpp::Publisher<omm_perception_msgs::msg::PerceptionHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double last_clock_ = -1.;
  std::atomic<bool> reset_pending_{false};
  rclcpp::JumpHandler::SharedPtr jump_handler_;
};
std::shared_ptr<rclcpp::Node> makeObstacleMemory(const rclcpp::NodeOptions &options) {
  return std::make_shared<ObstacleMemory>(options);
}
} // namespace omm_perception
