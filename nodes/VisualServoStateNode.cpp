#include "brick_pickup_sm/VisualServoStateMachine.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_servo_sm_node");
  ros::NodeHandle nh;

  std::shared_ptr<uav_reference::VisualServoStateMachine> vssmObj{
    new uav_reference::VisualServoStateMachine(nh)
  };
  vssmObj->run();
}