#include "uav_ros_control/reference/GlobalPickupStateMachine.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "brick_pickup_node");
  ros::NodeHandle nh;

  std::shared_ptr<uav_sm::GlobalPickupStateMachine> vssmObj{
    new uav_sm::GlobalPickupStateMachine(nh)
  };
  ros::spin();
}