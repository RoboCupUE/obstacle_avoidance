#include "bump_and_go/collision_detector.hpp"

#include <rclcpp/rclcpp.hpp>

#include <stdlib.h>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto collision_detector_node = std::make_shared<collision_detector::CollisionDetector>(
    "collision_detector_node");

  rclcpp::Rate rate(collision_detector_node->getRate());

  while(rclcpp::ok()) {
    rclcpp::spin_some(collision_detector_node);
    collision_detector_node->step();
    rate.sleep();
  }

  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}