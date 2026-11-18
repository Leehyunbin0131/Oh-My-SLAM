#include "omm_costmap_layers/keepout_layer.hpp"
#include <gtest/gtest.h>
#include <thread>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace {
using Snapshot=omslam_operator_msgs::msg::KeepoutZones;
class KeepoutTest:public ::testing::Test {
protected:
  static void SetUpTestSuite(){rclcpp::init(0,nullptr);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
  void SetUp() override {
    node=std::make_shared<nav2_util::LifecycleNode>("keepout_fixture");
    publisher=std::make_shared<rclcpp::Node>("keepout_source");
    buffer=std::make_unique<tf2_ros::Buffer>(node->get_clock());
    pub=publisher->create_publisher<Snapshot>("/operator/keepouts",rclcpp::QoS(1).reliable().transient_local());
    group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    global=std::make_unique<nav2_costmap_2d::LayeredCostmap>("map",false,false);
    local=std::make_unique<nav2_costmap_2d::LayeredCostmap>("odom",true,false);
    for(auto *map:{global.get(),local.get()})map->resizeMap(100,100,.05,-2.5,-2.5);
    map_layer=std::make_shared<omm_costmap_layers::KeepoutLayer>();
    odom_layer=std::make_shared<omm_costmap_layers::KeepoutLayer>();
    map_layer->initialize(global.get(),"global_keepout",buffer.get(),node,group);
    odom_layer->initialize(local.get(),"local_keepout",buffer.get(),node,group);
    global->addPlugin(map_layer);local->addPlugin(odom_layer);
    executor.add_node(node->get_node_base_interface());executor.add_node(publisher);
    auto end=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while(pub->get_subscription_count()<2&&std::chrono::steady_clock::now()<end){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}
    ASSERT_EQ(pub->get_subscription_count(),2u);transform(0.,0.);
  }
  void TearDown() override {
    executor.remove_node(node->get_node_base_interface());executor.remove_node(publisher);
    map_layer.reset();odom_layer.reset();global.reset();local.reset();buffer.reset();pub.reset();node.reset();publisher.reset();
  }
  void transform(double x,double yaw) {
    geometry_msgs::msg::TransformStamped t;t.header.stamp=node->now();
    t.header.frame_id="map";t.child_frame_id="odom";t.transform.translation.x=x;
    tf2::Quaternion q;q.setRPY(0,0,yaw);t.transform.rotation=tf2::toMsg(q);
    ASSERT_TRUE(buffer->setTransform(t,"fixture",false));
  }
  Snapshot sample(uint64_t revision=1) {
    Snapshot s;s.header.stamp=node->now();s.header.frame_id="map";s.session_id="map-a";s.revision=revision;
    geometry_msgs::msg::Polygon p;
    for(const auto &xy:std::vector<std::pair<double,double>>{{.5,-.1},{.8,-.1},{.8,.1},{.5,.1}}) {
      geometry_msgs::msg::Point32 q;q.x=xy.first;q.y=xy.second;p.points.push_back(q);
    }
    s.polygons.push_back(p);return s;
  }
  void send(const Snapshot &s,double robot_x=0.) {
    pub->publish(s);auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(30);
    while(std::chrono::steady_clock::now()<end){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    global->updateMap(0,0,0);local->updateMap(robot_x,0,0);
  }
  unsigned char cost(nav2_costmap_2d::LayeredCostmap &map,double x,double y) {
    unsigned int mx,my;EXPECT_TRUE(map.getCostmap()->worldToMap(x,y,mx,my));return map.getCostmap()->getCost(mx,my);
  }
  std::shared_ptr<nav2_util::LifecycleNode> node;
  rclcpp::Node::SharedPtr publisher;
  rclcpp::Publisher<Snapshot>::SharedPtr pub;
  std::unique_ptr<tf2_ros::Buffer> buffer;
  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> global,local;
  std::shared_ptr<omm_costmap_layers::KeepoutLayer> map_layer,odom_layer;
};
TEST_F(KeepoutTest, MarksFullAreaPersistsAcrossClearAndDeletesOldBounds) {
  EXPECT_FALSE(map_layer->isCurrent());
  send(sample());ASSERT_TRUE(map_layer->isCurrent());ASSERT_TRUE(odom_layer->isCurrent());
  EXPECT_EQ(cost(*global,.65,0),254);EXPECT_EQ(cost(*local,.65,0),254);
  EXPECT_FALSE(map_layer->isClearable());map_layer->reset();global->updateMap(0,0,0);EXPECT_EQ(cost(*global,.65,0),254);
  auto empty=sample(2);empty.polygons.clear();send(empty);
  EXPECT_EQ(cost(*global,.65,0),0);EXPECT_EQ(cost(*local,.65,0),0);
  send(sample(1));EXPECT_EQ(cost(*global,.65,0),0); // stale revision cannot resurrect
}
TEST_F(KeepoutTest, ReprojectsLocalRotationsAndRollingOrigins) {
  send(sample());transform(.2,M_PI/2);send(sample(2),.1);
  EXPECT_EQ(cost(*global,.65,0),254);
  EXPECT_EQ(cost(*local,0,-.45),254);EXPECT_EQ(cost(*local,.65,0),0);
  buffer->clear();local->updateMap(.1,0,0);
  EXPECT_FALSE(odom_layer->isCurrent());EXPECT_EQ(cost(*local,0,-.45),254);
}
TEST_F(KeepoutTest, InvalidSnapshotCannotEraseAndNewSessionCanStartEmpty) {
  send(sample());auto bad=sample(2);bad.polygons[0].points[0].x=NAN;send(bad);
  EXPECT_FALSE(map_layer->isCurrent());EXPECT_EQ(cost(*global,.65,0),254);
  auto next=sample(0);next.session_id="map-b";next.polygons.clear();send(next);
  EXPECT_TRUE(map_layer->isCurrent());EXPECT_EQ(cost(*global,.65,0),0);
}
} // namespace
