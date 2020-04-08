#include "brick_pickup_sm/GlobalPickupStateMachine.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "global_pickup_sm_node");
  ros::NodeHandle nh;

  std::shared_ptr<uav_sm::GlobalPickupStateMachine> vssmObj{
    new uav_sm::GlobalPickupStateMachine(nh)
  };
  ros::spin();
}