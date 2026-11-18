// Copyright 2026 Oh_My_SLAM contributors
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Actor.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace omslam
{
// Scripted people use animated actors for rendering / GPU sensors and a
// separate, top-level static model for physical contact. Actors themselves
// are deliberately not treated as physics bodies by Gazebo.
class AirportPedestrians final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
  struct Route
  {
    std::string actorName;
    std::string collisionName;
    double speed{1.0};
    double phase{0.0};
    double animationFactor{4.153902689};
    std::vector<gz::math::Vector2d> points;
    std::vector<double> lengths;
    double totalLength{0.0};
    gz::sim::Entity actor{gz::sim::kNullEntity};
    gz::sim::Entity collision{gz::sim::kNullEntity};
    bool reportedMissing{false};
  };

  public: void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &) override
  {
    this->world = gz::sim::World(_entity);
    try
    {
      const double updateRate = _sdf->Get<double>("update_rate", 30.0).first;
      if (!std::isfinite(updateRate) || updateRate <= 0.0)
        throw std::runtime_error("update_rate must be finite and positive");
      this->updatePeriod = 1.0 / updateRate;
      std::set<std::string> names;
      for (auto entry = _sdf->FindElement("pedestrian"); entry;
        entry = entry->GetNextElement("pedestrian"))
      {
        Route route;
        route.actorName = entry->Get<std::string>("actor");
        route.collisionName = entry->Get<std::string>("collision_model");
        route.speed = entry->Get<double>("speed", 1.0).first;
        route.phase = entry->Get<double>("phase", 0.0).first;
        route.animationFactor = entry->Get<double>("animation_factor", 4.153902689).first;
        if (route.actorName.empty() || route.collisionName.empty() ||
          !names.insert(route.actorName).second ||
          !names.insert(route.collisionName).second)
        {
          throw std::runtime_error("pedestrian entity names must be distinct");
        }
        if (!std::isfinite(route.speed) || route.speed <= 0.0 ||
          !std::isfinite(route.phase) || route.phase < 0.0 ||
          !std::isfinite(route.animationFactor) || route.animationFactor <= 0.0)
        {
          throw std::runtime_error("invalid speed, phase or animation_factor");
        }
        for (auto waypoint = entry->FindElement("waypoint"); waypoint;
          waypoint = waypoint->GetNextElement("waypoint"))
        {
          std::istringstream input(waypoint->Get<std::string>());
          double x{0.0}, y{0.0};
          std::string extra;
          if (!(input >> x >> y) || (input >> extra) ||
            !std::isfinite(x) || !std::isfinite(y))
          {
            throw std::runtime_error("waypoint must contain two finite metres");
          }
          const gz::math::Vector2d point(x, y);
          if (route.points.empty() ||
            point.Distance(route.points.back()) > 1e-8)
            route.points.push_back(point);
        }
        if (route.points.size() > 1 &&
          route.points.front().Distance(route.points.back()) <= 1e-8)
          route.points.pop_back();
        if (route.points.size() < 2)
          throw std::runtime_error("a closed route needs at least two points");
        for (std::size_t i = 0; i < route.points.size(); ++i)
        {
          const double length = route.points[i].Distance(
            route.points[(i + 1) % route.points.size()]);
          if (length <= 1e-8)
            throw std::runtime_error("route contains a zero-length segment");
          route.lengths.push_back(length);
          route.totalLength += length;
        }
        this->routes.push_back(std::move(route));
      }
      this->posePublisher = this->node.Advertise<gz::msgs::Pose_V>(
        "/airport/pedestrians/poses");
      if (!this->posePublisher.Valid())
        throw std::runtime_error("could not advertise pedestrian pose diagnostics");
      this->configured = true;
      gzmsg << "[AirportPedestrians] " << this->routes.size()
            << " deterministic routes; animation and collision proxies enabled.\n";
    }
    catch (const std::exception &error)
    {
      this->routes.clear();
      this->configured = false;
      gzerr << "[AirportPedestrians] Invalid configuration: "
            << error.what() << '\n';
    }
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured)
      return;
    const double seconds = std::chrono::duration<double>(_info.simTime).count();
    if (seconds < this->lastUpdate)
    {
      // World reset or time rewind: positions are an absolute function of
      // simulation time, not an accumulator tied to wall-clock scheduling.
      this->lastUpdate = -std::numeric_limits<double>::infinity();
      this->lastDiagnostic = -std::numeric_limits<double>::infinity();
    }
    if (std::isfinite(this->lastUpdate) &&
      (_info.paused || seconds - this->lastUpdate < this->updatePeriod - 1e-9))
      return;
    this->lastUpdate = seconds;
    for (auto &route : this->routes)
    {
      if (!gz::sim::Actor(route.actor).Valid(_ecm))
        route.actor = this->world.ActorByName(_ecm, route.actorName);
      if (!gz::sim::Model(route.collision).Valid(_ecm))
        route.collision = this->world.ModelByName(_ecm, route.collisionName);
      if (route.actor == gz::sim::kNullEntity ||
        route.collision == gz::sim::kNullEntity)
      {
        if (!route.reportedMissing)
        {
          gzerr << "[AirportPedestrians] Waiting for actor/proxy pair: "
                << route.actorName << " / " << route.collisionName << '\n';
          route.reportedMissing = true;
        }
        continue;
      }
      const double travel = std::max(0.0, seconds + route.phase) * route.speed;
      double remaining = std::fmod(travel, route.totalLength);
      std::size_t segment = 0;
      while (segment + 1 < route.lengths.size() &&
        remaining >= route.lengths[segment])
      {
        remaining -= route.lengths[segment];
        ++segment;
      }
      const auto &start = route.points[segment];
      const auto &end = route.points[(segment + 1) % route.points.size()];
      const double fraction = remaining / route.lengths[segment];
      const auto position = start + (end - start) * fraction;
      const double yaw = std::atan2(end.Y() - start.Y(), end.X() - start.X());
      const gz::math::Pose3d trajectory(
        position.X(), position.Y(), 0.0, 0.0, 0.0, yaw);

      gz::sim::Actor actor(route.actor);
      actor.SetTrajectoryPose(_ecm, trajectory);
      actor.SetAnimationName(_ecm, "walk");
      actor.SetAnimationTime(_ecm,
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(travel * route.animationFactor)));
      // Actor helper setters do not mark their data as changed. Explicit
      // notifications also update a separate GUI client, not only sensors.
      _ecm.SetChanged(route.actor, gz::sim::components::TrajectoryPose::typeId,
        gz::sim::ComponentState::OneTimeChange);
      _ecm.SetChanged(route.actor, gz::sim::components::AnimationName::typeId,
        gz::sim::ComponentState::OneTimeChange);
      _ecm.SetChanged(route.actor, gz::sim::components::AnimationTime::typeId,
        gz::sim::ComponentState::OneTimeChange);
      gz::sim::Model(route.collision).SetWorldPoseCmd(_ecm, trajectory);
    }
  }

  public: void PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured || !this->posePublisher.Valid())
      return;
    const double seconds = std::chrono::duration<double>(_info.simTime).count();
    if (std::isfinite(this->lastDiagnostic) &&
      (_info.paused || seconds - this->lastDiagnostic < 0.2 - 1e-9))
      return;
    this->lastDiagnostic = seconds;
    gz::msgs::Pose_V message;
    const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
      _info.simTime).count();
    message.mutable_header()->mutable_stamp()->set_sec(nanoseconds / 1000000000);
    message.mutable_header()->mutable_stamp()->set_nsec(nanoseconds % 1000000000);
    for (const auto &route : this->routes)
    {
      const gz::sim::Actor actor(route.actor);
      if (!actor.Valid(_ecm) || !gz::sim::Model(route.collision).Valid(_ecm))
        continue;
      const auto origin = actor.Pose(_ecm);
      const auto trajectory = actor.TrajectoryPose(_ecm);
      if (!origin || !trajectory)
        continue;
      auto *actorPose = message.add_pose();
      gz::msgs::Set(actorPose, *origin * *trajectory);
      actorPose->set_name(route.actorName);
      actorPose->set_id(route.actor);
      auto *collisionPose = message.add_pose();
      // Read the physical model after Physics processes WorldPoseCmd. This
      // diagnostic is not just another copy of the desired trajectory.
      gz::msgs::Set(collisionPose, gz::sim::worldPose(route.collision, _ecm));
      collisionPose->set_name(route.collisionName);
      collisionPose->set_id(route.collision);
    }
    this->posePublisher.Publish(message);
  }

  private: gz::sim::World world;
  private: std::vector<Route> routes;
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher posePublisher;
  private: bool configured{false};
  private: double updatePeriod{1.0 / 30.0};
  private: double lastUpdate{-std::numeric_limits<double>::infinity()};
  private: double lastDiagnostic{-std::numeric_limits<double>::infinity()};
};
}  // namespace omslam

GZ_ADD_PLUGIN(omslam::AirportPedestrians,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate,
  gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(omslam::AirportPedestrians, "omslam::AirportPedestrians")
