#include "omm_costmap_layers/obstacle_memory_layer.hpp"
#include <algorithm>
#include <cmath>
#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace omm_costmap_layers {
void ObstacleMemoryLayer::onInitialize() {
  auto node = node_.lock();
  if (!node)
    throw std::runtime_error("Costmap node unavailable");
  declareParameter("padding", rclcpp::ParameterValue(.06));
  declareParameter("sensor_timeout", rclcpp::ParameterValue(.30));
  declareParameter("memory_frame", rclcpp::ParameterValue(std::string("odom")));
  node->get_parameter(name_ + ".padding", padding_);
  node->get_parameter(name_ + ".sensor_timeout", timeout_);
  node->get_parameter(name_ + ".memory_frame", memory_frame_);
  if (!std::isfinite(padding_) || padding_ < 0 || !std::isfinite(timeout_) ||
      timeout_ <= 0 || memory_frame_.empty())
    throw std::invalid_argument("Invalid obstacle memory layer parameters");
  enabled_ = true;
  current_ = false;
  sub_ = node->create_subscription<omm_perception_msgs::msg::ObstacleMemory>(
      "/perception/memory", rclcpp::QoS(1).transient_local(),
      [this](omm_perception_msgs::msg::ObstacleMemory::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_ = msg;
      });
}

void ObstacleMemoryLayer::updateBounds(double, double, double, double *min_x,
                                      double *min_y, double *max_x, double *max_y) {
  // Nav2 resets this union in the master before applying every layer. Build
  // one projection here so bounds and costs always describe the same snapshot.
  // World-space bounds remain valid after a rolling origin shift; both old and
  // new map<-odom projections are included when localization changes.
  const auto grid = layered_costmap_->getCostmap();
  auto include = [&](const std::vector<tf2::Vector3> &points, double radius) {
    const double extent = radius + .5 * grid->getResolution();
    for (const auto &p : points) {
      *min_x = std::min(*min_x, p.x() - extent);
      *min_y = std::min(*min_y, p.y() - extent);
      *max_x = std::max(*max_x, p.x() + extent);
      *max_y = std::max(*max_y, p.y() + extent);
    }
  };
  include(transformed_, radius_);
  include(archived_, archive_radius_);
  prepareProjection();
  include(transformed_, radius_);
  include(archived_, archive_radius_);
}

void ObstacleMemoryLayer::prepareProjection() {
  omm_perception_msgs::msg::ObstacleMemory::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg = snapshot_;
  }
  const auto now = clock_->now();
  if (now.seconds() < last_now_) {
    // A new clock epoch cannot reuse occupancy in the previous odom frame.
    // updateBounds has already included its old marks for clearing.
    accepted_.reset();
    transformed_.clear();
    archived_.clear();
  }
  last_now_ = now.seconds();
  current_ = false;
  if (msg) {
    const auto observation = rclcpp::Time(msg->observation_stamp);
    const auto published = rclcpp::Time(msg->header.stamp);
    const double age = (now - observation).seconds();
    const double transport_age = (now - published).seconds();
    auto finite = [](const auto &p) {
      return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    };
    const bool ordered = !accepted_ ||
        (published >= rclcpp::Time(accepted_->header.stamp) &&
         observation >= rclcpp::Time(accepted_->observation_stamp));
    current_ = msg->sensor_healthy && msg->geometry_healthy && msg->memory_healthy &&
        age >= 0 && age < timeout_ && transport_age >= 0 && transport_age < timeout_ &&
        observation.nanoseconds() > 0 && observation <= published && ordered &&
        msg->header.frame_id == memory_frame_ &&
        std::isfinite(msg->voxel_size) && msg->voxel_size > 0 &&
        std::isfinite(msg->archive_resolution) && msg->archive_resolution >= 0 &&
        (msg->archived_points.empty() || msg->archive_resolution > 0) &&
        std::all_of(msg->points.begin(), msg->points.end(), finite) &&
        std::all_of(msg->archived_points.begin(), msg->archived_points.end(), finite);
    if (current_)
      accepted_ = msg;
  }
  // Unhealthy, stale, malformed or out-of-order snapshots cannot erase valid
  // occupancy. Preserve the last accepted source points and reproject them with
  // map<-odom; non-current prevents Nav2 from treating this as fresh sensing.
  if (accepted_) {
    try {
      auto transform = tf_->lookupTransform(layered_costmap_->getGlobalFrameID(),
                                            accepted_->header.frame_id,
                                            tf2::TimePointZero);
      tf2::Transform tf;
      tf2::fromMsg(transform.transform, tf);
      if (!std::isfinite(tf.getOrigin().x()) || !std::isfinite(tf.getOrigin().y()) ||
          !std::isfinite(tf.getRotation().length2()) || tf.getRotation().length2() < .25)
        throw std::runtime_error("Invalid memory transform");
      std::vector<tf2::Vector3> points, archived;
      points.reserve(accepted_->points.size());
      archived.reserve(accepted_->archived_points.size());
      for (const auto &p : accepted_->points)
        points.push_back(tf * tf2::Vector3(p.x, p.y, p.z));
      for (const auto &p : accepted_->archived_points)
        archived.push_back(tf * tf2::Vector3(p.x, p.y, p.z));
      transformed_ = std::move(points);
      archived_ = std::move(archived);
      radius_ = padding_ + .5 * accepted_->voxel_size;
      archive_radius_ = padding_ + std::sqrt(.5) * accepted_->archive_resolution;
    } catch (const std::exception &e) {
      current_ = false;
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,
                          "Cannot transform obstacle memory: %s", e.what());
      // Keep the previous successful projection until TF is available.
    }
  }
}

void ObstacleMemoryLayer::updateCosts(nav2_costmap_2d::Costmap2D &grid,
                                     int min_i, int min_j, int max_i, int max_j) {
  const double resolution = grid.getResolution();
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(int(grid.getSizeInCellsX()), max_i);
  max_j = std::min(int(grid.getSizeInCellsY()), max_j);
  auto draw = [&](const std::vector<tf2::Vector3> &points, double radius) {
    for (const auto &p : points) {
      int x, y;
      grid.worldToMapNoBounds(p.x(), p.y(), x, y);
      const int n = std::ceil(radius / resolution);
      for (int yy = std::max(y - n, min_j); yy < std::min(y + n + 1, max_j); ++yy)
        for (int xx = std::max(x - n, min_i); xx < std::min(x + n + 1, max_i); ++xx) {
          double wx, wy;
          grid.mapToWorld(xx, yy, wx, wy);
          if (std::hypot(wx - p.x(), wy - p.y()) <= radius + .5 * resolution)
            grid.setCost(xx, yy, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
    }
  };
  draw(transformed_, radius_);
  draw(archived_, archive_radius_);
}
} // namespace omm_costmap_layers
PLUGINLIB_EXPORT_CLASS(omm_costmap_layers::ObstacleMemoryLayer, nav2_costmap_2d::Layer)
