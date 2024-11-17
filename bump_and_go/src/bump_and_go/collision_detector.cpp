#include "bump_and_go/collision_detector.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <math.h>
#include <string>

using std::placeholders::_1;

namespace collision_detector
{
  CollisionDetector::CollisionDetector(const std::string & node_name)
  : Node(node_name), tf2StaticBroadcaster_(this)
  {
    initParams();

    laserSub_ = this->create_subscription<LaserScan>(laserTopic_, rclcpp::QoS(1),
      std::bind(&CollisionDetector::laserCallback, this, _1));
  }

  void CollisionDetector::step(void)
  {
    staticTransformBroadcast();
    publishCollisionData();
  }

  double CollisionDetector::getRate(void)
  {
    return rate_;
  }

  /*******************
   * Private Methods *
   *******************/

  void CollisionDetector::staticTransformBroadcast(void)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = fixedFrame_;
    t.child_frame_id = childFrame_;

    t.transform.translation.x = transformCoords_.at(0);
    t.transform.translation.y = transformCoords_.at(1);
    t.transform.translation.z = transformCoords_.at(2);

    tf2::Quaternion q;
    q.setRPY(
      transformCoords_.at(3),
      transformCoords_.at(4),
      transformCoords_.at(5));

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf2StaticBroadcaster_.sendTransform(t);
  }

  void CollisionDetector::publishCollisionData(void)
  {
    ;
  }

  void CollisionDetector::laserCallback(const LaserScan & msg)
  {
    scanMsg_ = msg;
    RCLCPP_INFO(this->get_logger(), "Received laser data");
  }

  void CollisionDetector::initParams(void)
  {
    transformCoords_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    leftRangeY_ = {0.0, 0.0};
    righRangeY_ = {0.0, 0.0};
    frontRangeY_ = {0.0, 0.0};

    rate_ = (double)this->declare_parameter("node_rate", 2.0);
    collisionDist_ = (double)this->declare_parameter("collision_distance", 0.3);
    laserTopic_ = (std::string)this->declare_parameter("laser_topic", "laser_scan");
    leftRangeY_ = (std::vector<double>)this->declare_parameter("left_range_y", leftRangeY_);
    righRangeY_ = (std::vector<double>)this->declare_parameter("right_range_y", righRangeY_);
    frontRangeY_ = (std::vector<double>)this->declare_parameter("front_range_y", frontRangeY_);
    fixedFrame_ = (std::string)this->declare_parameter("fixed_frame", "base_scan");
    childFrame_ = (std::string)this->declare_parameter("child_frame", "collision_frame");
    transformCoords_ = (std::vector<double>)this->declare_parameter("transform_coords", transformCoords_);
  }

} // end namespace collision_detector