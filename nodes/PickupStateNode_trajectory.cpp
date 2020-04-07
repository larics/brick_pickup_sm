#include "uav_ros_control/reference/PickupStateMachine.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vs_state_machine_node");
  ros::NodeHandle nh;

  std::shared_ptr<uav_reference::PickupStateMachine> vssmObj{
    new uav_reference::PickupStateMachine(nh)
  };
  ros::spin();
}