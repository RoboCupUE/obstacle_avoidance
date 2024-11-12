#include "bump_and_go/robot_mover.hpp"

#define VERBOSE
// #undef VERBOSE

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace robot_mover
{

RobotMover::RobotMover(const std::string & node_name)
: Node(node_name),
  collisionData_(eCollision_UNKNOWN),
  state_(eState_STOPPED),
  lastCollisionDataTs_(0)
{
  initParams();
  velPub_ = this->create_publisher<Twist>(velocityTopic_, rclcpp::QoS(1));
  collisionSub_ = this->create_subscription<Int32>(collisionTopic_,
    rclcpp::QoS(1), std::bind(&RobotMover::collisionCallback, this, _1));
  timer_ = this->create_wall_timer(1s, std::bind(&RobotMover::watchdog, this));
}

void RobotMover::step(void)
{
  applyTransition();

  runState();
}

/*******************
 * PRIVATE METHODS *
 *******************/

void RobotMover::applyTransition(void)
{
  int newState = -1;

  if (collisionData_ == eCollision_UNKNOWN) {
    state_ = eState_STOPPED;
#ifdef VERBOSE
    RCLCPP_WARN(this->get_logger(), "Applying security stop");
#endif
    return;
  }

  switch (state_)
  {
  case eState_STOPPED:
    if (collisionData_ == eCollision_NO_COLLISION) {
      newState = eState_FORWARD;
    } else if (collisionData_ == eCollision_CENTER) {
      newState = eState_TURNING_LEFT;
    } else if (collisionData_ == eCollision_LEFT) {
      newState = eState_TURNING_RIGHT;
    } else if (collisionData_ == eCollision_RIGHT) {
      newState = eState_TURNING_LEFT;
    }
    timeSecs_ = this->get_clock()->now().seconds();
    break;

  case eState_FORWARD:
    if (collisionData_ == eCollision_CENTER) {
      newState = eState_TURNING_LEFT;
    } else if (collisionData_ == eCollision_LEFT) {
      newState = eState_TURNING_RIGHT;
    } else if (collisionData_ == eCollision_RIGHT) {
      newState = eState_TURNING_LEFT;
    }
    timeSecs_ = this->get_clock()->now().seconds();
    break;

  case eState_TURNING_LEFT:
    if ((this->get_clock()->now().seconds() - timeSecs_) >= turningTime_) {
      newState = eState_STOPPED;
    }
    break;
  
  case eState_TURNING_RIGHT:
    if ((this->get_clock()->now().seconds() - timeSecs_) >= turningTime_) {
      newState = eState_STOPPED;
    }
    break;

  default:
    break;
  }

  if (newState != -1) {
#ifdef VERBOSE
  
    RCLCPP_INFO(this->get_logger(), "State transifion from [%d] to [%d]", state_, newState);
#endif
    state_ = (State_e)newState;
  }
}

void RobotMover::runState(void)
{
  switch (state_)
  {
  case eState_STOPPED:
    move(0.0, 0.0);
    break;

  case eState_FORWARD:
    move(linearVel_, 0.0);
    break;

  case eState_TURNING_LEFT:
    move(0.0, angularVel_);
    break;

  case eState_TURNING_RIGHT:
    move(0.0, -angularVel_);
    break;

  default:
    break;
  }
}

void RobotMover::move(double linearX, double angularZ)
{
  Twist velMsg_;

  velMsg_.linear.x = linearX;
  velMsg_.linear.y = velMsg_.linear.z = 0.0;

  velMsg_.angular.z = angularZ;
  velMsg_.angular.x = velMsg_.angular.y = 0.0;
  
  velPub_->publish(velMsg_);
}

void RobotMover::collisionCallback(const Int32 & msg)
{
  if ((msg.data > eCollision_UNKNOWN) & (msg.data <= eCollision_RIGHT)) {
    collisionData_ = (Collision_e)msg.data;
  } else {
    collisionData_ = eCollision_UNKNOWN;
#ifdef VERBOSE
    RCLCPP_WARN(this->get_logger(), "Unknown colission data");
#endif
  }
  lastCollisionDataTs_ = this->get_clock()->now();
}

void RobotMover::watchdog(void)
{
  double elapsedSecs;

  if (lastCollisionDataTs_.nanoseconds() == 0L)
    return;

  elapsedSecs = this->get_clock()->now().seconds() - lastCollisionDataTs_.seconds();

  if (elapsedSecs > watchdogTime_) {
    collisionData_ = eCollision_UNKNOWN;
  }
}

void RobotMover::initParams(void)
{
  velocityTopic_ = this->declare_parameter("velocity_topic", "cmd_vel");
  linearVel_ = (double)this->declare_parameter("linear_vel", 0.5);
  angularVel_ = (double)this->declare_parameter("angular_vel", 0.7);
  turningTime_ = (double)this->declare_parameter("turning_time", 1.0);
  watchdogTime_ = (double)this->declare_parameter("watchdog_time", 1.0);
}

} // end namespace robot_mover