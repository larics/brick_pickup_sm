#include "brick_pickup_sm/MasterPickupControl.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "master_pickup_sm_node");
  ros::NodeHandle nh;

  auto masterPickup = std::make_shared<uav_sm::MasterPickupControl>(nh);
  masterPickup->run(nh);
}