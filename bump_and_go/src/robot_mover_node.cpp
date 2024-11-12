#include "bump_and_go/robot_mover.hpp"

#include <rclcpp/rclcpp.hpp>

#include <stdlib.h>
#include <memory>

#define RATE  5  // Hz

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto mover_node = std::make_shared<robot_mover::RobotMover>("robot_mover_node");

  rclcpp::Rate rate(RATE);
  while (rclcpp::ok()) {
    rclcpp::spin_some(mover_node);
    mover_node->step();
    rate.sleep();
  }

  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}