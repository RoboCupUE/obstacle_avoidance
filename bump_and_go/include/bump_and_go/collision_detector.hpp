#ifndef COLLISION_DETECTOR_HPP_
#define COLLISION_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <string>

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
  void laserCallback(const LaserScan & msg);
  void initParams(void);

  rclcpp::Subscription<LaserScan>::SharedPtr laserSub_;

  // Configuration parameters
  double rate_;
  std::string laserTopic_;
};
} // end namespace collision_detector

#endif  // COLLISION_DETECTOR_HPP_