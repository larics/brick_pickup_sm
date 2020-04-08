# Brick Pickup State Machine

A collection of state machines for a UAV brick pickup scenario.

## State Machine Structure

Overall structure consits of four hierarchicaly connected state machines.

* **Master State Machine**
  * OFF
  * SEARCH - e.g. search a global area with a predefined pattern
  * ACTION - e.g. pickup and deliver brick
* **Global State Machine**
  * OFF
  * APPROACH
  * SEARCH - search local area around target global position
  * ATTEMPT_PICKUP
  * DROPOFF
* **Visual Servo State Machine**
  * OFF
  * BRICK_ALIGNMENT
  * DESCENT
  * TOUCHDOWN_ALIGNMENT
  * TOUCHDOWN

## Usage

### Prerequisites

This repository is meant to be used with the following packages.

* [uav_search](https://github.com/larics/uav-search-strategies/tree/mbzirc)
* [topp_ros](https://github.com/larics/topp_ros/tree/nuc-onboard)
* [color_filter](https://github.com/larics/MBZIRC_color_filter)
* [pointcloud_filter](https://github.com/larics/pointcloud_filter)
* [uav_ros_control](https://github.com/larics/uav_ros_control)
* [mbzirc_mission_control](https://github.com/mkrizmancic/mbzirc_mission_control)

All of the mentioned packages need to be installed in the catkin workspace before proceeding further.

### Launch Commands

First launch all the prerequisites in the following way.

```bash
roslaunch color_filter kopter_filter_servo.launch
roslaunch pointcloud_filter kopter_rs.launch
roslaunch uav_ros_control pid_carrot.launch
```

Secondly launch the visual servo stack as follows.

```bash
roslaunch brick_pickup_sm visual_servo_brick.launch
roslaunch brick_pickup_sm global_brick_pickup.launch
roslaunch brick_pickup_sm master_pickup_control.launch
```
