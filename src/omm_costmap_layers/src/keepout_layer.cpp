#include "omm_costmap_layers/keepout_layer.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace omm_costmap_layers {
namespace {
using Polygon = std::vector<tf2::Vector3>;
bool overlaps(const Polygon &a, const Polygon &b) {
  for (const auto *polygon : {&a, &b}) {
    for (size_t i=0; i<polygon->size(); ++i) {
      const auto &p = (*polygon)[i], &q = (*polygon)[(i+1)%polygon->size()];
      const double ax=p.y()-q.y(), ay=q.x()-p.x();
      double alo=INFINITY, ahi=-INFINITY, blo=INFINITY, bhi=-INFINITY;
      for (const auto &v:a) {double d=v.x()*ax+v.y()*ay; alo=std::min(alo,d); ahi=std::max(ahi,d);}
      for (const auto &v:b) {double d=v.x()*ax+v.y()*ay; blo=std::min(blo,d); bhi=std::max(bhi,d);}
      if (ahi < blo-1e-9 || bhi < alo-1e-9) return false;
    }
  }
  return true;
}
bool valid(const omslam_operator_msgs::msg::KeepoutZones &m) {
  if (m.header.frame_id!="map" || m.session_id.empty() || m.polygons.size()>64) return false;
  for (const auto &p:m.polygons) {
    if(p.points.size()!=4) return false;
    double x0=INFINITY,x1=-INFINITY,y0=INFINITY,y1=-INFINITY;
    for(const auto &v:p.points) {
      if(!std::isfinite(v.x)||!std::isfinite(v.y)||!std::isfinite(v.z)||
         std::abs(v.x)>10000||std::abs(v.y)>10000||std::abs(v.z)>1e-6) return false;
      x0=std::min(x0,double(v.x));x1=std::max(x1,double(v.x));
      y0=std::min(y0,double(v.y));y1=std::max(y1,double(v.y));
    }
    auto minimum_width = [](double lo, double hi) {
      const float bound = static_cast<float>(std::max(std::abs(lo),std::abs(hi)));
      const double spacing = std::nextafter(bound, std::numeric_limits<float>::infinity()) - bound;
      return .05 - std::clamp(spacing, 1e-6, .0001);
    };
    if (x1-x0 < minimum_width(x0,x1) || y1-y0 < minimum_width(y0,y1)) return false;
    // Require canonical order; malformed/bow-tie geometry cannot clear marks.
    const double xs[]={x0,x1,x1,x0},ys[]={y0,y0,y1,y1};
    for(size_t i=0;i<4;++i)
      if(p.points[i].x!=xs[i]||p.points[i].y!=ys[i]) return false;
  }
  return true;
}
} // namespace

void KeepoutLayer::onInitialize() {
  auto node=node_.lock();
  if(!node) throw std::runtime_error("Keepout node unavailable");
  enabled_=true;current_=false;
  sub_=node->create_subscription<omslam_operator_msgs::msg::KeepoutZones>(
    "/operator/keepouts",rclcpp::QoS(1).reliable().transient_local(),
    [this](omslam_operator_msgs::msg::KeepoutZones::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      if(snapshot_ && msg->session_id==snapshot_->session_id && msg->revision<snapshot_->revision) return;
      snapshot_=msg;
    });
}

void KeepoutLayer::updateBounds(double,double,double,double *min_x,double *min_y,double *max_x,double *max_y) {
  const double margin=layered_costmap_->getCostmap()->getResolution();
  auto include=[&]() {
    for(const auto &polygon:projected_) for(const auto &p:polygon) {
      *min_x=std::min(*min_x,p.x()-margin);*max_x=std::max(*max_x,p.x()+margin);
      *min_y=std::min(*min_y,p.y()-margin);*max_y=std::max(*max_y,p.y()+margin);
    }
  };
  include(); // Old marks must be included when zones move or are deleted.
  omslam_operator_msgs::msg::KeepoutZones::ConstSharedPtr msg;
  {std::lock_guard<std::mutex> lock(mutex_);msg=snapshot_;}
  current_=false;
  if(msg && valid(*msg)) {
    try {
      tf2::Transform transform;transform.setIdentity();
      if(!msg->polygons.empty() && layered_costmap_->getGlobalFrameID()!=msg->header.frame_id) {
        auto stamped=tf_->lookupTransform(layered_costmap_->getGlobalFrameID(),msg->header.frame_id,tf2::TimePointZero);
        const double age=(clock_->now()-rclcpp::Time(stamped.header.stamp)).seconds();
        if(rclcpp::Time(stamped.header.stamp).nanoseconds()!=0 && (age>0.5||age < -1.0)) throw std::runtime_error("Keepout transform is stale");
        tf2::fromMsg(stamped.transform,transform);
        if(!std::isfinite(transform.getOrigin().x())||!std::isfinite(transform.getOrigin().y())||
           !std::isfinite(transform.getRotation().length2())||transform.getRotation().length2()<.25)
          throw std::runtime_error("Invalid keepout transform");
      }
      std::vector<Polygon> next;
      for(const auto &p:msg->polygons) {
        Polygon polygon;
        for(const auto &v:p.points) polygon.push_back(transform*tf2::Vector3(v.x,v.y,0.));
        next.push_back(std::move(polygon));
      }
      projected_=std::move(next);current_=true;
    } catch(const std::exception &e) {
      RCLCPP_WARN_THROTTLE(logger_,*clock_,5000,"Keepout transform unavailable: %s",e.what());
      // Preserve existing occupancy and stop consumers through non-current.
    }
  }
  include();
}

void KeepoutLayer::updateCosts(nav2_costmap_2d::Costmap2D &grid,int min_i,int min_j,int max_i,int max_j) {
  const double half=.5*grid.getResolution();
  for(const auto &polygon:projected_) {
    double x0=INFINITY,x1=-INFINITY,y0=INFINITY,y1=-INFINITY;
    for(const auto &p:polygon) {x0=std::min(x0,p.x());x1=std::max(x1,p.x());y0=std::min(y0,p.y());y1=std::max(y1,p.y());}
    int a,b,c,d;grid.worldToMapNoBounds(x0-half,y0-half,a,b);grid.worldToMapNoBounds(x1+half,y1+half,c,d);
    a=std::max({a,min_i,0});b=std::max({b,min_j,0});
    c=std::min({c+1,max_i,int(grid.getSizeInCellsX())});d=std::min({d+1,max_j,int(grid.getSizeInCellsY())});
    for(int j=b;j<d;++j) for(int i=a;i<c;++i) {
      double x,y;grid.mapToWorld(i,j,x,y);
      Polygon cell={{x-half,y-half,0},{x+half,y-half,0},{x+half,y+half,0},{x-half,y+half,0}};
      if(overlaps(polygon,cell)) grid.setCost(i,j,nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}
} // namespace omm_costmap_layers
PLUGINLIB_EXPORT_CLASS(omm_costmap_layers::KeepoutLayer,nav2_costmap_2d::Layer)
