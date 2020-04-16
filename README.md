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

## Prerequisites

### Build dependencies

* [uav_search](https://github.com/larics/uav-search-strategies/tree/mbzirc)
* [topp_ros](https://github.com/larics/topp_ros/tree/nuc-onboard)
* [color_filter](https://github.com/larics/MBZIRC_color_filter)
* [pointcloud_filter](https://github.com/larics/pointcloud_filter)
* [uav_ros_control](https://github.com/larics/uav_ros_control)
* [mbzirc_mission_control](https://github.com/mkrizmancic/mbzirc_mission_control)

All of the mentioned packages need to be installed in the catkin workspace before proceeding further.

### Simulation dependencies

The following packages are used for a Software-In-The-Loop brick pickup scenario in the Gazebo simulation environemnt.

* [ardupilot_gazebo](https://github.com/larics/ardupilot_gazebo/tree/kopterworx_setup_mbzirc)
* [ardupilot](https://github.com/ArduPilot/ardupilot)
* [storm_gazebo_ros_magnet](https://github.com/larics/storm_gazebo_ros_magnet/tree/melodic_electromagnet_dev)

### Real-World dependencies

The following pakcages are meant to be used with an actual Mavros compatible UAV with a USB connected magnet toggle switch.  

* [multirotor_transformations](https://github.com/larics/multirotor_transformations/tree/erl_uav_master)
* [brick_detector](https://github.com/larics/brick_detector)
* [erl_drone](https://github.com/larics/erl_drone)

## Usage

Make sure to add set UAV_NAMESPACE variable in *.bashrc*.

```bash
echo "export UAV_NAMESPACE=red" >> ~/.bashrc
```

Launch all the prerequisites in the following way.

```bash
roslaunch color_filter kopter_filter_servo.launch
roslaunch pointcloud_filter kopter_rs.launch
roslaunch uav_ros_control pid_carrot.launch manual_takeoff:=false
```

Launch the brick pickup stack as follows.

```bash
roslaunch brick_pickup_sm visual_servo_brick.launch
roslaunch brick_pickup_sm global_brick_pickup.launch
roslaunch brick_pickup_sm master_pickup_control.launch
```

To start the brick pickup scenario call the following ROS service.

```bash
rosservice call /$UAV_NAMESPACE/brick_pickup/master "data: true"
```

### Software-In-The-Loop

To start the SITL simulation run the following commands.

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris -m --mav10 --console -I0
roslaunch ardupilot_gazebo bebop.launch enable_velodyne:=false enbale_magnet:=true
roslaunch ardupilot_gazebo mavros.launch
rosrun ardupilot_gazebo brick_spawn.sh
roslaunch brick_pickup_sm magnet_simulation.launch
```

Before initializing brick pickup make sure to run the following commands in the MAVProxy terminal.

```bash
mode GUIDED
set streamreate 50
```

### Hardware-In-The-Loop

Connect to the UAV onboard computer and run the following commands.

```bash
roslaunch erl_drone apm2.launch
rosrun erl_drone mavsys_script.sh
```
