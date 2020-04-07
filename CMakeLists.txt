cmake_minimum_required(VERSION 3.10.0)
project(brick_pickup_sm)

find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  config/VisualServoStateMachineParameters.cfg
  config/GlobalPickupStateMachineParameters.cfg
  config/MasterPickupStateMachineParameters.cfg)

catkin_package(INCLUDE_DIRS LIBRARIES CATKIN_DEPENDS)
include_directories(include ${catkin_INCLUDE_DIRS})
