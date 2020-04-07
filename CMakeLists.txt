cmake_minimum_required(VERSION 3.10.0)
project(brick_pickup_sm)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             uav_ros_control_msgs
             dynamic_reconfigure
             mavros
             color_filter
             larics_motion_planning
             uav_search
             mbzirc_mission_control)

list(INSERT CMAKE_MODULE_PATH 0 "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)
find_package(cmake_modules REQUIRED)

# Find Eigen
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

generate_dynamic_reconfigure_options(
  config/control/PositionControlParameters.cfg
  config/control/DistanceControlParameters.cfg
  config/reference/VisualServoParameters.cfg
  config/reference/VisualServoStateMachineParameters.cfg
  config/reference/GlobalPickupStateMachineParameters.cfg
  config/reference/MasterPickupStateMachineParameters.cfg)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  my_pid
  nfilt
  my_kalman
  CATKIN_DEPENDS
  uav_ros_control_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${GeographicLib_INCLUDE_DIRS})

# PID controller library
add_library(my_pid src/control/PID.cpp)
target_link_libraries(my_pid ${catkin_LIBRARIES})

# Nonlinear filters library
add_library(nfilt src/filters/NonlinearFilters.cpp)
target_link_libraries(nfilt ${catkin_LIBRARIES})

# Kalman filter library
add_library(my_kalman src/filters/KalmanFilter.cpp)
target_link_libraries(my_kalman ${catkin_LIBRARIES})

# References library
file(GLOB_RECURSE reference_SRC "src/reference/*.cpp")
add_library(reference_lib ${reference_SRC})
target_link_libraries(reference_lib ${catkin_LIBRARIES} nfilt my_pid yaml-cpp)

# Controller library
file(GLOB_RECURSE controllers_SRC "src/control/*.cpp")
add_library(control_lib ${controllers_SRC})
target_link_libraries(control_lib ${catkin_LIBRARIES} nfilt)

# Carrot node
add_executable(carrot_reference_node nodes/reference/CarrotReferenceNode.cpp)
target_link_libraries(carrot_reference_node reference_lib)

# Visual servo node
add_executable(visual_servo_node nodes/reference/VisualServoNode.cpp)
target_link_libraries(visual_servo_node reference_lib)

add_executable(visual_servo_idle nodes/reference/VisualServoIdleNode.cpp)
target_link_libraries(visual_servo_idle reference_lib)

# visual servo state machine
add_executable(visual_servo_sm nodes/reference/VisualServoStateNode.cpp)
target_link_libraries(visual_servo_sm ${catkin_LIBRARIES} nfilt)

# Global pickup state machine
add_executable(brick_pickup_sm nodes/reference/GlobalPickupStateNode.cpp)
target_link_libraries(brick_pickup_sm ${catkin_LIBRARIES} nfilt
                      ${GeographicLib_LIBRARIES})

# pickup state machine - with trajectories
add_executable(pickup_sm nodes/reference/PickupStateNode_trajectory.cpp)
target_link_libraries(pickup_sm ${catkin_LIBRARIES} nfilt)

# master pickup state machnie
add_executable(master_brick_pickup nodes/reference/MasterPickupControlNode.cpp)
target_link_libraries(master_brick_pickup ${catkin_LIBRARIES} nfilt)

# PID cascade node
add_executable(pid_cascade_node nodes/control/PositionControlNode.cpp)
target_link_libraries(pid_cascade_node control_lib)

# Distance control node
add_executable(distance_control_node nodes/control/DistanceControlNode.cpp)
target_link_libraries(distance_control_node control_lib nfilt)

# Geo-fencing node
add_executable(geo_fence_node nodes/reference/GeoFenceNode.cpp)
target_link_libraries(geo_fence_node reference_lib ${catkin_LIBRARIES} nfilt)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
