#include "bump_and_go/collision_detector.hpp"

#include <math.h>
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

  int CollisionDetector::angle2laserIndex(float rad, const LaserScan & scanMsg)
  {
    double angle = rad;
    /**
     * Move 'angle' to a range [0 - 2PI]
     */
    while (angle >= (2.0 * M_PI)) {
      angle -= (2.0 * M_PI);
    }

    size_t index = 0;
    bool found = false;
    while (index < scanMsg.ranges.size()) { 
      if ((scanMsg.angle_min + ((float)index * scanMsg.angle_increment)) >= rad) {
        found = true;
        break;
      }
      index++;
    }

    if (found) {
      return index;
    } else {
      return -1;
    }

  }

} // end namespace collision_detector