#ifndef VISUAL_SERVO_STATE_MACHINE_H
#define VISUAL_SERVO_STATE_MACHINE_H

#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <uav_ros_control/GlobalPickupStateMachineParametersConfig.h>
#include <uav_ros_control_msgs/GeoBrickApproach.h>
#include <uav_ros_control/filters/Util.hpp>
#include <iostream>
#include <uav_ros_control/reference/Global2Local.hpp>
#include <uav_ros_control/reference/TrajectoryGenerator.hpp>
#include <nav_msgs/Odometry.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <uav_ros_control/reference/PickupStates.hpp>

using namespace pickup_states;
using namespace ros_util;

namespace uav_sm {

struct BrickPickupStatus
{
  BrickPickupStatus() : BrickPickupStatus("red", Eigen::Vector3d(0, 0, 0)) {}
  BrickPickupStatus(const std::string &t_brickColor,
    const Eigen::Vector3d &t_brickLocal,
    const GlobalPickupStates t_status = GlobalPickupStates::OFF)
    : m_brickColor(t_brickColor), m_status(t_status), m_localBrick(t_brickLocal),
      m_dropoffPos(-1, -1, -1), m_dropoffPositionSet(false)
  {}

  bool isOff() { return m_status == GlobalPickupStates::OFF; }

  bool isSearching() { return m_status == GlobalPickupStates::SEARCH; }

  bool isApproaching() { return m_status == GlobalPickupStates::APPROACH; }

  bool isAttemptingPickup() { return m_status == GlobalPickupStates::ATTEMPT_PICKUP; }

  bool isDropOff() { return m_status == GlobalPickupStates::DROPOFF; }

  void setDropoffPosition(Eigen::Vector3d &&vec)
  {
    m_dropoffPositionSet = true;
    m_dropoffPos = std::move(vec);
  }

  bool isDropoffPositionSet() { return m_dropoffPositionSet; }

  bool m_dropoffPositionSet;
  GlobalPickupStates m_status;
  std::string m_brickColor;
  Eigen::Vector3d m_localBrick, m_dropoffPos;
};

typedef uav_ros_control::GlobalPickupStateMachineParametersConfig PickupParams;
typedef uav_ros_control_msgs::GeoBrickApproach GeoBrickMsg;
typedef GeoBrickMsg::Request GeoBrickReq;
typedef GeoBrickMsg::Response GeoBrickResp;

class GlobalPickupStateMachine
{

public:
  GlobalPickupStateMachine(ros::NodeHandle &t_nh)
    : m_handlerVSSMState(t_nh, "visual_servo_sm/state"),
      m_handlerOdometry(t_nh, "odometry"), m_handlerBrickAttached(t_nh, "brick_attached"),
      m_handlerTrajectoryStatus(t_nh, "topp/status"),
      m_handlerCurrentTrajectory(t_nh, "carrot/trajectory"),
      m_handlerPatchCount(t_nh, "n_contours"), m_global2Local(t_nh)
  {
    initialize_parameters(t_nh);

    // Initialize publisher
    m_pubTrajGen = t_nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      "topp/input/trajectory", 1);
    m_pubGlobalPickupStatus = t_nh.advertise<std_msgs::Int32>("global_pickup/status", 1);

    // Initialize service callers
    m_vssmCaller =
      t_nh.serviceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
        "brick_pickup/local");
    m_magnetOverrideOnCaller =
      t_nh.serviceClient<std_srvs::Empty::Request, std_srvs::Empty::Response>(
        "magnet/override_ON");
    m_magnetOverrideOffCaller =
      t_nh.serviceClient<std_srvs::Empty::Request, std_srvs::Empty::Response>(
        "magnet/override_OFF");
    m_pickupSuccessCaller =
      t_nh.serviceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
        "brick_pickup/success");

    // Advertise service
    m_serviceBrickPickup = t_nh.advertiseService("brick_pickup/global",
      &uav_sm::GlobalPickupStateMachine::brick_pickup_global_cb,
      this);

    // Iniitalize timers
    m_stateTimer = t_nh.createTimer(
      ros::Duration(1.0 / getParamOrThrow<double>(t_nh, "brick_pickup/update_rate")),
      &uav_sm::GlobalPickupStateMachine::update_state,
      this);
    m_publishTrajectoryTimer = t_nh.createTimer(
      ros::Duration(1.0 / getParamOrThrow<double>(t_nh, "brick_pickup/pub_traj_rate")),
      &uav_sm::GlobalPickupStateMachine::publish_trajectory,
      this);
  }

private:
  inline const LocalPickupState getCurrentVisualServoState()
  {
    return static_cast<LocalPickupState>(m_handlerVSSMState.getData().data);
  }

  void update_state(const ros::TimerEvent & /* unused */)
  {
    std_msgs::Int32 stateMsg;
    stateMsg.data = static_cast<int>(m_currentStatus.m_status);
    m_pubGlobalPickupStatus.publish(stateMsg);

    if (m_currentStatus.isApproaching() && is_close_to_brick()) {
      ROS_WARN("BrickPickup::update_state - SEARCH state activated");
      m_currentStatus.m_status = GlobalPickupStates::SEARCH;
      m_searchRadius = m_pickupConfig->getData().initial_search_radius;
      clear_current_trajectory();
      return;
    }

    if (m_currentStatus.isSearching()
        && getCurrentVisualServoState() == LocalPickupState::OFF
        && m_handlerPatchCount.getData().data
             > 0// THere are some patches visible, so activation is possible
        && toggle_visual_servo_state_machine(true)) {
      ROS_WARN("BrickPickup::update_state - ATTEMPT_PICKUP activated");
      m_currentStatus.m_status = GlobalPickupStates::ATTEMPT_PICKUP;
      toggle_magnet(true);
      clear_current_trajectory();
      return;
    }

    if (m_currentStatus.isAttemptingPickup()
        && getCurrentVisualServoState() == LocalPickupState::OFF) {
      ROS_WARN("BrickPickup::update_state - VisualServoState is OFF!.");
      m_searchRadius = m_pickupConfig->getData().initial_search_radius;

      clear_current_trajectory();
      if (is_brick_picked_up()) {

        // Update both dropoff and brick pickup global position with current relative
        // height
        ros::Duration(m_pickupConfig->getData().after_pickup_sleep).sleep();
        ros::spinOnce();
        m_currentStatus.m_dropoffPos.z() =
          m_handlerOdometry.getData().pose.pose.position.z;
        m_currentStatus.m_localBrick.z() =
          m_handlerOdometry.getData().pose.pose.position.z;

        // Refresh dropoff position
        set_dropoff_position(m_currentStatus.m_dropoffPos.z());
        ROS_INFO(
          "BrickPickup::update_state - brick is picked up, DROPOFF state activated.");
        m_currentStatus.m_status = GlobalPickupStates::DROPOFF;

      } else {
        ROS_FATAL(
          "BrickPickup::update_state - brick is not picked up, ATTEMPT_PICKUP state "
          "activated");
        m_currentStatus.m_status = GlobalPickupStates::SEARCH;
        advertise_pickup_success(false);
      }
      return;
    }

    // Case when we drop off brick (either on purpose or intentionally)
    if (m_currentStatus.isDropOff() && !is_brick_picked_up()) {
      ROS_WARN("BrickPickup::update_state - DROPOFF finished, APPROACH activated");
      m_currentStatus.m_status = GlobalPickupStates::APPROACH;
      clear_current_trajectory();
      toggle_magnet(false);
      advertise_pickup_success(true);// TODO: Sometimes mighnt not be successful
      return;
    }

    if (m_currentStatus.isDropOff() && is_brick_picked_up() && is_close_to_dropoff()) {
      ROS_INFO("BrickPickup::updateState - at DROPOFF position");
      toggle_magnet(false);
      clear_current_trajectory();
      return;
    }
  }

  void publish_trajectory(const ros::TimerEvent & /* unused */)
  {
    if (m_currentStatus.isSearching() && !is_trajectory_active()) {
      ROS_INFO(
        "BrickPickup - generating SEARCH trajectory with radius: %.3f.", m_searchRadius);
      m_pubTrajGen.publish(uav_reference::traj_gen::generateCircleTrajectory_topp(
        m_currentStatus.m_localBrick.x(),
        m_currentStatus.m_localBrick.y(),
        m_currentStatus.m_localBrick.z(),
        m_handlerCurrentTrajectory.getData(),
        20, /* Number of points */
        m_searchRadius));
      m_searchRadius += m_pickupConfig->getData().search_readius_increment;
      return;
    }

    if (m_currentStatus.isApproaching() && !is_close_to_brick()
        && !is_trajectory_active()) {
      ROS_INFO("BrickPickup - generating APPROACH trajectory.");
      m_pubTrajGen.publish(uav_reference::traj_gen::generateLinearTrajectory_topp(
        m_currentStatus.m_localBrick.x(),
        m_currentStatus.m_localBrick.y(),
        m_currentStatus.m_localBrick.z(),
        m_handlerCurrentTrajectory.getData()));
      return;
    }

    if (m_currentStatus.isDropOff() && !is_close_to_dropoff()
        && !is_trajectory_active()) {
      ROS_INFO("BrickPickup - generating DROPOFF trajectory.");
      m_pubTrajGen.publish(uav_reference::traj_gen::generateLinearTrajectory_topp(
        m_currentStatus.m_dropoffPos.x(),
        m_currentStatus.m_dropoffPos.y(),
        m_currentStatus.m_dropoffPos.z(),
        m_handlerCurrentTrajectory.getData()));
    }
  }

  bool brick_pickup_global_cb(GeoBrickReq &request, GeoBrickResp &response)
  {
    // If we want to disable the global brick pickup
    if (!request.enable || !all_services_available()) {// TODO: Add checks for services
      ROS_FATAL("BPSM::brick_pickup_global_cb - brick_pickup/global disabled");
      response.status = false;
      m_currentStatus = BrickPickupStatus();
      toggle_visual_servo_state_machine(false);
      toggle_magnet(false);
      return true;
    }

    // Enable global brick pickup
    m_currentStatus = BrickPickupStatus(request.brick_color,
      m_global2Local.toLocal(
        request.latitude, request.longitude, request.altitude_relative, true),
      GlobalPickupStates::APPROACH);

    set_dropoff_position(request.altitude_relative);
    ROS_INFO("Current brick goal: [%.3f, %.3f, %.3f]",
      m_currentStatus.m_localBrick.x(),
      m_currentStatus.m_localBrick.y(),
      m_currentStatus.m_localBrick.z());

    clear_current_trajectory();
    response.status = true;
    return true;
  }

  void set_dropoff_position(double altitude)
  {
    double dropoffLat, dropoffLon;
    bool gotLat = m_nh.getParam("brick_dropoff/lat", dropoffLat);
    bool gotLon = m_nh.getParam("brick_dropoff/lon", dropoffLon);
    ROS_FATAL_COND(!gotLat || !gotLon, "BrickPickup - dropoff position unavailable");

    if (gotLat && gotLon) {
      // If brick_dropoff is available set its position
      m_currentStatus.setDropoffPosition(
        m_global2Local.toLocal(dropoffLat, dropoffLon, altitude, true));
    } else {
      // Otherwise set dropoff to zero - TODO: Have a better backupf
      m_currentStatus.setDropoffPosition(Eigen::Vector3d(0, 0, altitude));
    }
  }

  void initialize_parameters(ros::NodeHandle &nh)
  {
    PickupParams initParams;
    getParamOrThrow(
      nh, "brick_pickup/brick_approach_tolerance", initParams.brick_approach_tolerance);
    getParamOrThrow(nh,
      "brick_pickup/dropoff_approach_tolerance",
      initParams.dropoff_approach_tolerance);
    getParamOrThrow(nh, "brick_pickup/after_pickup_sleep", initParams.after_pickup_sleep);
    getParamOrThrow(
      nh, "brick_pickup/initial_search_radius", initParams.initial_search_radius);
    getParamOrThrow(
      nh, "brick_pickup/search_readius_increment", initParams.search_readius_increment);
    m_pickupConfig.reset(
      new ParamHandler<PickupParams>(initParams, "brick_config/brick_pickup"));
  }

  bool all_services_available()
  {
    ROS_FATAL_COND(!m_vssmCaller.exists(), "GlobalPickup - VSSM service non exsistant.");
    ROS_FATAL_COND(!m_magnetOverrideOnCaller.exists(),
      "GlobalPickup - magnet override does not exist.");
    ROS_FATAL_COND(!m_magnetOverrideOffCaller.exists(),
      "GlobalPickup - magnet override does not exist.");
    return m_vssmCaller.exists() && m_magnetOverrideOnCaller.exists()
           && m_magnetOverrideOffCaller.exists();
  }

  bool toggle_magnet(bool t_magnetOn = true)
  {
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response resp;

    bool magnetToggled = false;
    if (t_magnetOn) {
      magnetToggled = m_magnetOverrideOnCaller.call(req, resp);
    } else {
      magnetToggled = m_magnetOverrideOffCaller.call(req, resp);
    }

    if (!magnetToggled) {
      ROS_FATAL("GlobalPickup::toggle_magnet - unable to toggle magnet.");
      return false;
    }

    ROS_INFO("GlobalPickupt::toggle_magnet - magnet toggled");
    return true;
  }

  bool toggle_visual_servo_state_machine(bool t_enable = true)
  {
    std_srvs::SetBool::Request req;
    std_srvs::SetBool::Response resp;
    req.data = t_enable;

    if (!m_vssmCaller.call(req, resp)) {
      ROS_FATAL("BrickPickup - unable to activate VSSM");
      return false;
    }
    ROS_INFO_COND(resp.success, "BrickPickup - VSSM activated");
    return resp.success;
  }

  bool is_close_to_brick()
  {
    return uav_reference::traj_gen::isCloseToReference(
      uav_reference::traj_gen::toTrajectoryPointMsg(m_currentStatus.m_localBrick.x(),
        m_currentStatus.m_localBrick.y(),
        m_currentStatus.m_localBrick.z(),
        0),
      m_handlerOdometry.getData(),
      m_pickupConfig->getData().brick_approach_tolerance);
  }

  bool is_close_to_dropoff()
  {
    return uav_reference::traj_gen::isCloseToReference(
      uav_reference::traj_gen::toTrajectoryPointMsg(m_currentStatus.m_dropoffPos.x(),
        m_currentStatus.m_dropoffPos.y(),
        m_currentStatus.m_dropoffPos.z(),
        0),
      m_handlerOdometry.getData(),
      m_pickupConfig->getData().dropoff_approach_tolerance);
  }

  bool is_brick_picked_up() { return m_handlerBrickAttached.getData().data == true; }

  void clear_current_trajectory()
  {
    ROS_WARN("clear_current_trajectory - Clearing trajectory");
    m_pubTrajGen.publish(trajectory_msgs::MultiDOFJointTrajectory());
    ros::Duration(1.0).sleep();
  }

  bool is_trajectory_active() { return m_handlerTrajectoryStatus.getData().data; }

  void advertise_pickup_success(bool success)
  {
    std_srvs::SetBool::Request req;
    std_srvs::SetBool::Response resp;
    req.data = success;
    if (!m_pickupSuccessCaller.call(req, resp)) {
      ROS_FATAL(
        "GlobalPickup::advertise_pickup_success - unable to call brick_pickup/success "
        "service");
      return;
    }
  }

  double m_searchRadius;// Initialized at initializeParameters
  Global2Local m_global2Local;
  BrickPickupStatus m_currentStatus;
  std::unique_ptr<ParamHandler<PickupParams>> m_pickupConfig;

  ros::ServiceServer m_serviceBrickPickup;
  ros::ServiceClient m_vssmCaller, m_magnetOverrideOnCaller, m_magnetOverrideOffCaller,
    m_pickupSuccessCaller;

  ros::NodeHandle m_nh;
  ros::Publisher m_pubTrajGen, m_pubGlobalPickupStatus;
  ros::Timer m_stateTimer, m_initFilterTimer, m_publishTrajectoryTimer;
  TopicHandler<std_msgs::Int32> m_handlerVSSMState, m_handlerPatchCount;
  TopicHandler<nav_msgs::Odometry> m_handlerOdometry;
  TopicHandler<std_msgs::Bool> m_handlerBrickAttached;
  TopicHandler<std_msgs::Bool> m_handlerTrajectoryStatus;
  TopicHandler<trajectory_msgs::MultiDOFJointTrajectoryPoint> m_handlerCurrentTrajectory;
};
}// namespace uav_sm
#endif
