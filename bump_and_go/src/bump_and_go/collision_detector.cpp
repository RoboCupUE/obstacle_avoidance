#include "bump_and_go/collision_detector.hpp"

#include <string>

using std::placeholders::_1;

namespace collision_detector
{
  CollisionDetector::CollisionDetector(const std::string & node_name)
  : Node(node_name)
  {
    initParams();

    laserSub_ = this->create_subscription<LaserScan>(laserTopic_, rclcpp::QoS(1),
      std::bind(&CollisionDetector::laserCallback, this, _1));
  }

  void CollisionDetector::step(void)
  {
    ;
  }

  double CollisionDetector::getRate(void)
  {
    return rate_;
  }

  /*******************
   * Private Methods *
   *******************/

  void CollisionDetector::laserCallback(const LaserScan & msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received laser data");
  }

  void CollisionDetector::initParams(void)
  {
    rate_ = (double)this->declare_parameter("node_rate", 2.0);
    laserTopic_ = (std::string)this->declare_parameter("laser_topic", "laser_scan");
  }

} // end namespace collision_detector