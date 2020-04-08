cmake_minimum_required(VERSION 3.10.0)
project(brick_pickup_sm)

find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure uav_ros_control
                                        uav_ros_control_msgs)

# Find Eigen
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

list(INSERT CMAKE_MODULE_PATH 0 "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

generate_dynamic_reconfigure_options(
  config/VisualServoStateMachineParameters.cfg
  config/GlobalPickupStateMachineParameters.cfg
  config/MasterPickupStateMachineParameters.cfg config/VisualServoParameters.cfg)

catkin_package(INCLUDE_DIRS LIBRARIES CATKIN_DEPENDS)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# Visual servo node
#add_executable(visual_servo_node nodes/VisualServoNode.cpp src/VisualServo.cpp)
#target_link_libraries(visual_servo_node ${catkin_LIBRARIES})
#add_dependencies(visual_servo_node ${catkin_EXPORTED_TARGETS})

# visual servo state machine
add_executable(visual_servo_state_sm nodes/VisualServoStateNode.cpp)
target_link_libraries(visual_servo_state_sm ${catkin_LIBRARIES})
add_dependencies(visual_servo_state_sm ${catkin_EXPORTED_TARGETS})

# Global pickup state machine
#add_executable(brick_pickup_sm nodes/GlobalPickupStateNode.cpp)
#target_link_libraries(brick_pickup_sm ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})
#add_dependencies(brick_pickup_sm ${catkin_EXPORTED_TARGETS})

# master pickup state machnie add_executable(master_brick_pickup
# nodes/reference/MasterPickupControlNode.cpp) target_link_libraries(master_brick_pickup
# ${catkin_LIBRARIES} nfilt)
