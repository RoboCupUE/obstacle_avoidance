#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>

#include <string>

#ifndef ROBOT_MOVER_HPP_
#define ROBOT_MOVER_HPP_

using namespace geometry_msgs::msg;
using namespace std_msgs::msg;

namespace robot_mover
{

typedef enum {
  eCollision_UNKNOWN = 0,
  eCollision_NO_COLLISION,
  eCollision_LEFT,
  eCollision_CENTER,
  eCollision_RIGHT,
} Collision_e;

typedef enum {
  eState_STOPPED = 0,
  eState_FORWARD,
  eState_TURNING_LEFT,
  eState_TURNING_RIGHT,
} State_e;

class RobotMover : public rclcpp::Node
{
public:
  RobotMover(const std::string & node_name);
  
  void step(void);
private:
  void collisionCallback(const Int32 & msg);
  void applyTransition(void);
  void runState(void);
  void move(double linearX, double angularZ);
  void initParams(void);

  rclcpp::Publisher<Twist>::SharedPtr velPub_;
  rclcpp::Subscription<Int32>::SharedPtr collisionSub_;

  Collision_e collisionData_;
  State_e state_;
  double timeSecs_;

  // Configuration parameters

  std::string velocityTopic_;
  double linearVel_;
  double angularVel_;
  double turningTime_;
  const std::string collisionTopic_ = "collision_data";
};

} // end namespace robot_mover

#endif // ROBOT_MOVER_HPP_