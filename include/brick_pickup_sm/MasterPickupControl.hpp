#ifndef MASTER_PICKUP_CONTROL_H
#define MASTER_PICKUP_CONTROL_H

#include <ros/ros.h>
#include <uav_ros_control/filters/Util.hpp>
#include <uav_ros_control/reference/PickupStates.hpp>
#include <uav_ros_control/reference/TrajectoryGenerator.hpp>
#include <uav_ros_control/reference/Global2Local.hpp>
#include <Eigen/Dense>

#include <mavros_msgs/State.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <uav_search/GetPoints.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <uav_ros_control/MasterPickupStateMachineParametersConfig.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <uav_ros_control_msgs/TakeOff.h>
#include <mbzirc_mission_control/CompletedTask.h>
#include <mbzirc_mission_control/NextTask.h>
#include <uav_ros_control_msgs/GeoBrickApproach.h>
#include <color_filter/color.h>

using namespace ros_util;
using namespace pickup_states;
using namespace uav_reference;
typedef mbzirc_mission_control::NextTask::Response CurrentTask;
typedef uav_ros_control::MasterPickupStateMachineParametersConfig MasterConfig;

namespace uav_sm {

class PickupChallengeInfo
{
public:
  bool isBrickLocationSet() { return m_brickLocationFound; }

  bool isDropoffLocationSet() { return m_dropoffLocationFound; }

  void setDropoffLocation(Eigen::Vector3d dropoffLocation)
  {
    m_dropoffLocationFound = true;
    m_dropoffLocation = dropoffLocation;
  }

  void setBrickLocation(Eigen::Vector3d brickLocation)
  {
    m_brickLocationFound = true;
    m_brickLocation = brickLocation;
  }

  void setCurrentTask(CurrentTask newTask)
  {
    m_taskCompleted = false;
    m_currentTask = newTask;
  }

  const CurrentTask &getCurrentTask() { return m_currentTask; }

  bool isTaskValid() { return !m_currentTask.task_id.empty(); }

  bool isCurrentTaskCompleted() { return m_taskCompleted; }

  bool setTaskCompleted(bool status) { m_taskCompleted = status; }

  const Eigen::Vector3d &getBrickLocation() { return m_brickLocation; }

private:
  CurrentTask m_currentTask;
  bool m_brickLocationFound = false, m_dropoffLocationFound = false,
       m_taskCompleted = true;
  Eigen::Vector3d m_brickLocation{ -1, -1, -1 }, m_dropoffLocation{ -1, -1, -1 };
};

class MasterPickupControl
{
public:
  MasterPickupControl(ros::NodeHandle &t_nh)
    : m_handlerState(t_nh, "mavros/state"), m_handlerCarrotStatus(t_nh, "carrot/status"),
      m_handlerOdometry(t_nh, "mavros/global_position/local"),
      m_handlerPatchCount(t_nh, "n_contours"),
      m_handlerBrickGlobalStatus(t_nh, "global_pickup/status"),
      m_handlerTrajectoryStatus(t_nh, "topp/status"),
      m_handlerTrajectoryPoint(t_nh, "carrot/trajectory"),
      m_handlerBrickEnu(t_nh, "global_centroid_point"), m_globalToLocal(t_nh),
      m_currentState(MasterPickupStates::OFF)
  {
    initialize_parameters(t_nh);
    m_pubTrajGen = t_nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      "topp/input/trajectory", 1);
    m_pubMasterState = t_nh.advertise<std_msgs::Int32>("master_pickup/state", 1);
    m_serviceMasterPickup = t_nh.advertiseService(
      "brick_pickup/master", &uav_sm::MasterPickupControl::master_pickup_cb, this);
    m_servicePickupSuccess = t_nh.advertiseService(
      "brick_pickup/success", &uav_sm::MasterPickupControl::pickup_success_cb, this);

    // TODO: Setup timer containing challenge duration

    m_clientArming = t_nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    m_clientTakeoff = t_nh.serviceClient<uav_ros_control_msgs::TakeOff>("takeoff");
    m_clientLand = t_nh.serviceClient<std_srvs::SetBool>("land");
    m_clientSearchGenerator = t_nh.serviceClient<uav_search::GetPoints>("get_points");
    m_clientRequestTask =
      t_nh.serviceClient<mbzirc_mission_control::NextTask>("request_next_task");
    m_clientCompleteTask = t_nh.serviceClient<mbzirc_mission_control::CompletedTask>(
      "register_completed_task");
    m_clientGlobalPickup =
      t_nh.serviceClient<uav_ros_control_msgs::GeoBrickApproach>("brick_pickup/global");
    m_chooseColorCaller =
      t_nh.serviceClient<color_filter::color::Request, color_filter::color::Response>(
        "set_color");
  }

  void run(ros::NodeHandle &t_nh)
  {
    double rate = getParamOrThrow<double>(t_nh, "master_pickup/state_timer_rate");
    ros::Rate loopRate(rate);

    while (ros::ok()) {
      ros::spinOnce();
      run_internal();
      loopRate.sleep();
    }
  }

private:
  inline static void sleep_for(double duration) { ros::Duration(duration).sleep(); }
  inline bool valid_brick_position()
  {
    return m_handlerBrickEnu.getData().x != -1 && m_handlerBrickEnu.getData().y != -1
           && m_handlerBrickEnu.getData().z != -1;
  }
  inline bool is_trajectory_active() { return m_handlerTrajectoryStatus.getData().data; }
  inline bool is_brick_visible() { return m_handlerPatchCount.getData().data > 0; }
  inline bool is_guided_active()
  {
    return m_handlerState.getData().mode == "GUIDED_NOGPS";
  }
  inline bool is_land_active() { return m_handlerState.getData().mode == "LAND"; }
  inline bool is_vehicle_armed() { return m_handlerState.getData().armed; }
  inline bool in_off_state() const { return m_currentState == MasterPickupStates::OFF; }
  inline bool in_search_state() const
  {
    return m_currentState == MasterPickupStates::SEARCH;
  }
  inline bool in_task_state() const
  {
    return m_currentState == MasterPickupStates::ACTION;
  }
  inline bool is_uav_airborne()
  {
    return m_handlerCarrotStatus.getData().data == "CARROT_ON_AIR"
           && m_handlerCarrotStatus.getData().data == "HOLD";
  }
  inline bool is_in_global_pickup()
  {
    return static_cast<GlobalPickupStates>(m_handlerBrickGlobalStatus.getData().data)
           != GlobalPickupStates::OFF;
  }

  void initialize_parameters(ros::NodeHandle &t_nh)
  {
    MasterConfig config;
    getParamOrThrow(t_nh, "master_pickup/search_size_x", config.search_size_x);
    getParamOrThrow(t_nh, "master_pickup/search_size_y", config.search_size_y);
    getParamOrThrow(t_nh, "master_pickup/search_spacing", config.search_spacing);
    getParamOrThrow(
      t_nh, "master_pickup/search_yaw_correction", config.search_yaw_correction);
    getParamOrThrow(t_nh, "master_pickup/search_offset_x", config.search_offset_x);
    getParamOrThrow(t_nh, "master_pickup/search_offset_y", config.search_offset_y);
    getParamOrThrow(t_nh, "master_pickup/takeoff_height", config.takeoff_height);
    getParamOrThrow(t_nh, "master_pickup/search_height", config.search_height);
    getParamOrThrow(t_nh, "master_pickup/goto_home_tol", config.goto_home_tol);
    m_masterConfig.reset(
      new ParamHandler<MasterConfig>(config, "brick_config/master_pickup"));
  }

  bool all_services_available()
  {
    ROS_FATAL_COND(!m_clientArming.exists(),
      "MasterPickupControl - [mavros/cmd/arming] service does not exist.");
    ROS_FATAL_COND(!m_clientTakeoff.exists(),
      "MasterPickupControl - [takeoff] service does not exists");
    ROS_FATAL_COND(
      !m_clientLand.exists(), "MasterPickupControl - [land] service does not exist");
    ROS_FATAL_COND(!m_clientSearchGenerator.exists(),
      "MasterPickupControl - [get_points] service does not exist");
    ROS_FATAL_COND(!m_clientRequestTask.exists(),
      "MasterPickupControl - [request_next_task] service does not exist");
    ROS_FATAL_COND(!m_clientCompleteTask.exists(),
      "MasterPickupControl - [register_completed_task] service does not exist.");
    ROS_FATAL_COND(!m_clientGlobalPickup.exists(),
      "MasterPickupControl - [brick_pickup/global] service does not exist.");
    ROS_FATAL_COND(!m_chooseColorCaller.exists(),
      "MasterPickupControl - [set_color] service does not exist.");

    return m_clientArming.exists() && m_clientTakeoff.exists() && m_clientLand.exists()
           && m_clientSearchGenerator.exists() && m_clientRequestTask.exists()
           && m_clientCompleteTask.exists() && m_clientGlobalPickup.exists();
  }

  bool pickup_success_cb(std_srvs::SetBool::Request &request,
    std_srvs::SetBool::Response & /*unused*/)
  {
    const auto task_successful = [&request]() { return request.data; };

    if (task_successful()) {
      ROS_INFO_STREAM("MasterPickupControll::pickup_success_cb - ["
                      << m_challengeInfo.getCurrentTask().task_id << "] task sucessful.");
      m_challengeInfo.setTaskCompleted(true);
      register_completed_task();
      return true;
    }

    ROS_FATAL(
      "MasterPickupControl::pickup_success_cb - unsucessful pickup attempt registered.");
    // TODO: Do something when pickup fails, maybe disable global pickup and go back to
    // search
    return true;
  }

  void run_internal()
  {
    // Always try to publish current state
    std_msgs::Int32 stateMsg;
    stateMsg.data = static_cast<int>(m_currentState);
    m_pubMasterState.publish(stateMsg);

    // If master pickup is activated, disable everything if(when) Antun takes over :)
    if (!in_off_state() && !is_guided_active()) { switch_to_off_state(); }

    // Check if we see any bricks
    if (in_search_state() && !m_challengeInfo.isBrickLocationSet() && is_brick_visible()
        && valid_brick_position()) {
      auto brickGeoPos = m_globalToLocal.toGlobal(m_handlerBrickEnu.getData().x,
        m_handlerBrickEnu.getData().y,
        m_handlerBrickEnu.getData().z);
      ROS_INFO("MasterPickupControl::state_timer - BRICK seen at [%.10f, %.10f, %.10f]",
        brickGeoPos.x(),
        brickGeoPos.y(),
        brickGeoPos.z());
      m_challengeInfo.setBrickLocation(Eigen::Vector3d{ brickGeoPos.x(),
        brickGeoPos.y(),
        m_handlerOdometry.getData().pose.pose.position.z });
    }

    // Check if search trajectory needs regenerating
    if (in_search_state() && !is_trajectory_active() && !is_in_global_pickup()) {
      ROS_INFO(
        "MasterPickupControl::state_timer_cb - in SEARCH state, generate trajectory");
      generate_search_trajectory();
    }

    // If brick location is found start the mission
    if (in_search_state() && m_challengeInfo.isBrickLocationSet()) {
      ROS_INFO(
        "MasterPickupControl::state_timer_cb - in SEARCH state switching to task_state.");
      switch_to_task_state();
    }

    // Try to get new task
    if (in_task_state() && m_challengeInfo.isCurrentTaskCompleted()) {
      ROS_INFO(
        "MasterPickupControl::state_timer_cb - in TASK state, generating new task.");
      generate_new_task();
      toggle_global_pickup();
    }

    // ... or try to dispatch the current task
    if (in_task_state() && !is_in_global_pickup()
        && m_challengeInfo.isBrickLocationSet()) {
      ROS_INFO(
        "MasterPickupControl::state_timer_cb - in TASK state, entering global pickup");
      toggle_global_pickup();
    }
  }

  void generate_new_task()
  {
    if (!m_challengeInfo.isCurrentTaskCompleted()) {
      ROS_FATAL(
        "MasterPickupControl::generate_new_taask - uncompleted task pending,  generation "
        "denied.");
      return;
    }

    mbzirc_mission_control::NextTask::Request request;
    mbzirc_mission_control::NextTask::Response response;
    if (!m_clientRequestTask.call(request, response)) {
      ROS_FATAL("MasterPickupControl::generate_new_task - unable to request task.");
      m_challengeInfo.setCurrentTask(CurrentTask());
      m_challengeInfo.setTaskCompleted(true);
      return;
    }

    ROS_INFO_STREAM("MasterPickupControl::generate_new_task - ["
                    << response.task_id << "] - color: " << response.color);
    m_challengeInfo.setCurrentTask(response);

    // Check if current task is valid
    if (!m_challengeInfo.isTaskValid()) {
      // Invalid task we are finished with challenge :D
      // TODO: Test this behaviour
      ROS_WARN(
        "MAsterPickupControl:toggle_global_pickup - challenge FINISHED, going HOME.");
      switch_to_off_state();
      go_to_home();
      land_uav();
    }
  }

  void register_completed_task()
  {
    if (!m_challengeInfo.isCurrentTaskCompleted()) {
      ROS_FATAL(
        "MasterPickupControl::register_completed_task - unable to register an unfinished "
        "task");
      return;
    }

    mbzirc_mission_control::CompletedTask::Request request;
    mbzirc_mission_control::CompletedTask::Response response;
    request.success = true;
    request.task_id = m_challengeInfo.getCurrentTask().task_id;
    if (!m_clientCompleteTask.call(request, response)) {
      ROS_FATAL(
        "MasterPickupControl::register_completed_task - unable to call complete task "
        "service.");
      return;
    }

    // TODO: Do something with responses here
  }

  void toggle_global_pickup(bool enablePickup = true)
  {
    // If we are trying to enter global pickup, and task ID is invalid (for some reason),
    // do not enter
    if (enablePickup && !m_challengeInfo.isTaskValid()) {
      ROS_FATAL("MAsterPickupControl:toggle_global_pickup - invalid task ID");
      return;
    }

    // Set color if pickup is enabled
    if (enablePickup) { filter_choose_color(m_challengeInfo.getCurrentTask().color); }

    uav_ros_control_msgs::GeoBrickApproach::Request request;
    uav_ros_control_msgs::GeoBrickApproach::Response response;
    request.enable = enablePickup;
    request.brick_color = m_challengeInfo.getCurrentTask().color;
    request.latitude = m_challengeInfo.getBrickLocation().x();
    request.longitude = m_challengeInfo.getBrickLocation().y();
    request.altitude_relative =
      m_challengeInfo.getBrickLocation().z();// TODO: Pub current odometry here
    if (!m_clientGlobalPickup.call(request, response)) {
      ROS_FATAL(
        "MasterPickupControl::toggle_global_pickup - unable to call "
        "brick_pickup/global.");
      return;
    }

    if (response.status) {
      ROS_INFO(
        "MasterPickupControl::toggle_global_pickup - successfuly entered "
        "brick_pickup/global.");
    } else {
      ROS_INFO(
        "MasterPickupControl::toggle_global_pickup - enetering brick_pickup/global "
        "failed.");
    }
  }

  bool master_pickup_cb(std_srvs::SetBool::Request &request,
    std_srvs::SetBool::Response &response)
  {
    const auto deactivation_requested = [&request]() { return !request.data; };
    const auto set_response = [&response](bool success) { response.success = success; };

    if (deactivation_requested() && in_off_state()) {
      ROS_FATAL("MasterPickupControl - deactivation requested, already in OFF state.");
      toggle_global_pickup(false);
      set_response(false);
      return true;
    }

    if (!all_services_available() && in_off_state()) {
      ROS_FATAL("MasterPickupControl - service check failed.");
      switch_to_off_state();
      set_response(false);
      return true;
    }

    if (deactivation_requested() && !in_off_state()) {
      ROS_INFO("MasterPickupControl - deactivation requested.");
      switch_to_off_state();
      go_to_home();
      land_uav();
      set_response(false);
      return true;
    }

    // Assume mission is requested at this point
    if (!is_guided_active()) {
      ROS_FATAL("MasterPickupControl - request denied, not in GUIDED_NOGPS");
      set_response(false);
      return true;
    }

    if (is_vehicle_armed()) {
      ROS_FATAL("MasterPickupControl - request denied, vehicle is already ARMED");
      set_response(false);
      return true;
    }

    if (!arm_uav()) {
      ROS_FATAL("MasterPickupControl - request denied, ARMING failed.");
      set_response(false);
      return true;
    }
    sleep_for(POST_ARM_SLEEP);

    // Assume vehicle is armed at this point
    if (!takeoff_uav()) {
      ROS_FATAL("MasterPickupControl - request denied, TAKEOFF unsuccessful");
      set_response(false);
      return true;
    }
    sleep_for(POST_TAEKOFF_SLEEP);

    // Assume takeoff is successful at this point
    ROS_INFO("MasterPickupControl - request approved, TAKEOFF successful.");
    switch_to_search_state();
    set_response(true);
    return true;
  }

  void switch_to_off_state()
  {
    ROS_WARN_STREAM(m_currentState << " -> " << MasterPickupStates::OFF);
    m_currentState = MasterPickupStates::OFF;
    toggle_global_pickup(false);
    clear_current_trajectory();
  }

  void switch_to_search_state()
  {
    ROS_WARN_STREAM(m_currentState << " -> " << MasterPickupStates::SEARCH);
    clear_current_trajectory();
    filter_choose_color("all");
    ros::spinOnce();
    m_currentState = MasterPickupStates::SEARCH;
  }

  void switch_to_task_state()
  {
    ROS_WARN_STREAM(m_currentState << " -> " << MasterPickupStates::ACTION);
    clear_current_trajectory();
    m_currentState = MasterPickupStates::ACTION;
  }

  void generate_search_trajectory()
  {
    uav_search::GetPoints::Request pointsRequest;
    uav_search::GetPoints::Response pointsResponse;
    pointsRequest.request.height = m_masterConfig->getData().search_height;
    pointsRequest.request.pattern = "lawn";
    pointsRequest.request.size_x = m_masterConfig->getData().search_size_x;
    pointsRequest.request.size_y = m_masterConfig->getData().search_size_y;
    pointsRequest.request.spacing = m_masterConfig->getData().search_spacing;

    if (!m_clientSearchGenerator.call(pointsRequest, pointsResponse)) {
      ROS_FATAL(
        "MasterPickupcontrol::generate_search_trajectory - unable to call TRAJECTORY "
        "generator.");
      return;
    }

    const auto valid_trajectory = [&pointsResponse]() {
      return pointsResponse.response.size_x * pointsResponse.response.size_y
             == pointsResponse.response.data.size();
    };
    if (!valid_trajectory()) {
      ROS_FATAL(
        "MasterPickupControl::generate_search_trajectory - invalid trajectory recieved.");
      return;
    }

    trajectory_msgs::MultiDOFJointTrajectory searchtrajectory;
    searchtrajectory.header.stamp = ros::Time::now();
    searchtrajectory.points.push_back(m_handlerTrajectoryPoint.getData());

    for (std::size_t i = 0; i < pointsResponse.response.data.size();
         i += pointsResponse.response.size_y) {

      const double newX =
        pointsResponse.response.data[i] + m_masterConfig->getData().search_offset_x;
      const double newY =
        pointsResponse.response.data[i + 1] + m_masterConfig->getData().search_offset_y;

      double angle = m_masterConfig->getData().search_yaw_correction;
      const double newX_rot = newX * cos(angle) + newY * sin(angle);
      const double newY_rot = -newX * sin(angle) + newY * cos(angle);

      tf2::Quaternion q = traj_gen::getHeadingQuaternion(
        searchtrajectory.points.back().transforms[0].translation.x,
        searchtrajectory.points.back().transforms[0].translation.y,
        newX_rot,
        newY_rot);

      // Quick hack for quaternion
      searchtrajectory.points.push_back(traj_gen::toTrajectoryPointMsg(
        searchtrajectory.points.back().transforms[0].translation.x,
        searchtrajectory.points.back().transforms[0].translation.y,
        m_masterConfig->getData().search_height,
        q.getX(),
        q.getY(),
        q.getZ(),
        q.getW()));

      searchtrajectory.points.push_back(traj_gen::toTrajectoryPointMsg(newX_rot,
        newY_rot,
        m_masterConfig->getData().search_height,
        q.getX(),
        q.getY(),
        q.getZ(),
        q.getW()));
    }// end for
    clear_current_trajectory();
    ROS_INFO("MasterPickup - generating search trajectory.");
    m_pubTrajGen.publish(searchtrajectory);
    ros::Duration(2.0).sleep();
  }

  void clear_current_trajectory()
  {
    ROS_WARN("clear_current_trajectory - Clearing trajectory");
    m_pubTrajGen.publish(trajectory_msgs::MultiDOFJointTrajectory());
    ros::Duration(1.0).sleep();
  }

  void go_to_home()
  {
    ros::spinOnce();
    ROS_INFO("MasterPickupControl::go_to_home");
    double homeAltitude = m_handlerOdometry.getData().pose.pose.position.z;
    m_pubTrajGen.publish(traj_gen::generateLinearTrajectory_topp(
      0, 0, homeAltitude, m_handlerOdometry.getData()));

    const auto is_close_to_home = [this, &homeAltitude]() {
      return traj_gen::isCloseToReference(
        traj_gen::toTrajectoryPointMsg(0, 0, homeAltitude, 0),
        this->m_handlerOdometry.getData(),
        this->m_masterConfig->getData().goto_home_tol);
    };

    while (!is_close_to_home()) {
      ROS_INFO("MasterPickupControl::go_to_home - going home ...");
      ros::Duration(1.0).sleep();
      // TODO: This might be an issue if multiple calls happen at the same time
      ros::spinOnce();// Spin once to check odometry
    }

    // Arrived home
    clear_current_trajectory();
  }

  bool arm_uav()
  {
    ROS_WARN("MasterPickupControl - arming UAV.");
    mavros_msgs::CommandBool::Request armRequest;
    mavros_msgs::CommandBool::Response armResponse;
    armRequest.value = true;
    if (!m_clientArming.call(armRequest, armResponse)) {
      ROS_FATAL("MasterPickupControl::arm_uav - call to ARM service failed");
      return false;
    }

    if (!armResponse.success) {
      ROS_FATAL("MasterPickupControl::arm_uav - UAV arm failed");
      return false;
    }

    ROS_INFO("MasterPickupControl::arm_uav - UAV ARM successful");
    return true;
  }

  bool takeoff_uav()
  {
    // Clear trajectory in case something is generating
    clear_current_trajectory();

    ROS_WARN("MasterPickupControl - UAV takeoff to %.2f",
      m_masterConfig->getData().takeoff_height);
    uav_ros_control_msgs::TakeOff::Request takeoffRequest;
    uav_ros_control_msgs::TakeOff::Response takeoffResponse;
    takeoffRequest.rel_alt = m_masterConfig->getData().takeoff_height;
    if (!m_clientTakeoff.call(takeoffRequest, takeoffResponse)) {
      ROS_FATAL("MasterPickupControl::takeoff_uav - call to TAKEOFF service failed.");
      return false;
    }

    if (!takeoffResponse.success) {
      ROS_FATAL("MasterPickupControl::takeoff_uav - TAKEOFF failed.");
      return false;
    }
    ROS_INFO("MasterPickupControl::takeoff_uav - TAKEOFF successful");
    return true;
  }

  void land_uav()
  {
    ROS_WARN("MasterPickupControl - UAV LAND requested");
    std_srvs::SetBool::Request landRequest;
    std_srvs::SetBool::Response landResponse;
    landRequest.data = true;
    if (!m_clientLand.call(landRequest, landResponse)) {
      ROS_FATAL("MasterPickupControl::land_uav - call to LAND service failed");
      return;
    }

    if (!landResponse.success) {
      ROS_FATAL("MasterPickupControl::land_uav - LAND request failed.");
      return;
    }
    ROS_INFO("MasterPickupControl::laun_uav - LAND request succesfful.");
  }

  bool filter_choose_color(const std::string &t_color)
  {
    color_filter::color::Request req;
    color_filter::color::Response resp;
    req.color = t_color;

    if (!m_chooseColorCaller.call(req, resp)) {
      ROS_FATAL("MasterPickup - color initialization failed");
      return false;
    }

    // At this point color_initialization is assumed to be finished
    ROS_INFO("MasterPickup - to %s successful", t_color.c_str());
    return true;
  }

  static constexpr double POST_ARM_SLEEP = 3.0;
  static constexpr double POST_TAEKOFF_SLEEP = 10.0;

  Global2Local m_globalToLocal;
  PickupChallengeInfo m_challengeInfo;
  MasterPickupStates m_currentState;
  std::unique_ptr<ParamHandler<MasterConfig>> m_masterConfig;

  ros::ServiceServer m_serviceMasterPickup, m_servicePickupSuccess;
  ros::ServiceClient m_clientArming, m_clientTakeoff, m_clientLand,
    m_clientSearchGenerator, m_clientRequestTask, m_clientCompleteTask,
    m_clientGlobalPickup, m_chooseColorCaller;

  ros::Publisher m_pubTrajGen, m_pubMasterState;
  TopicHandler<mavros_msgs::State> m_handlerState;
  TopicHandler<std_msgs::String> m_handlerCarrotStatus;
  TopicHandler<nav_msgs::Odometry> m_handlerOdometry;
  TopicHandler<std_msgs::Int32> m_handlerPatchCount;
  TopicHandler<std_msgs::Int32> m_handlerBrickGlobalStatus;
  TopicHandler<std_msgs::Bool> m_handlerTrajectoryStatus;
  TopicHandler<geometry_msgs::Vector3> m_handlerBrickEnu;
  TopicHandler<trajectory_msgs::MultiDOFJointTrajectoryPoint> m_handlerTrajectoryPoint;
};

}// namespace uav_sm

#endif /* MASTER_PICKUP_CONTROL_H */