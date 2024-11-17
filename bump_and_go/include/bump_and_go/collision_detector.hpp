#ifndef COLLISION_DETECTOR_HPP_
#define COLLISION_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include "tf2_ros/static_transform_broadcaster.h"

#include <string>
#include <vector>

using namespace sensor_msgs::msg;

namespace collision_detector
{
class CollisionDetector : public rclcpp::Node
{
public:
  CollisionDetector(const std::string & node_name);

  void step(void);
  double getRate(void);
private:
  void staticTransformBroadcast(void);
  void publishCollisionData(void);
  void laserCallback(const LaserScan & msg);
  void initParams(void);

  rclcpp::Subscription<LaserScan>::SharedPtr laserSub_;
  LaserScan scanMsg_;
  tf2_ros::StaticTransformBroadcaster tf2StaticBroadcaster_;

  // Configuration parameters
  double rate_;
  double collisionDist_;
  std::string laserTopic_;
  std::vector<double> leftRangeY_;
  std::vector<double> righRangeY_;
  std::vector<double> frontRangeY_;
  std::string fixedFrame_;
  std::string childFrame_;
  std::vector<double> transformCoords_;
};
} // end namespace collision_detector

#endif  // COLLISION_DETECTOR_HPP_