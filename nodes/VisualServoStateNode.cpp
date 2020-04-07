#include "uav_ros_control/reference/VisualServoStateMachine.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vs_state_machine_node");
  ros::NodeHandle nh;

  std::shared_ptr<uav_reference::VisualServoStateMachine> vssmObj{
    new uav_reference::VisualServoStateMachine(nh)
  };
  vssmObj->run();
}