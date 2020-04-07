#include "uav_ros_control/reference/MasterPickupControl.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "master_pickup_node");
  ros::NodeHandle nh;

  auto masterPickup = std::make_shared<uav_sm::MasterPickupControl>(nh);
  masterPickup->run(nh);
}