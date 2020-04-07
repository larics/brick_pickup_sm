#ifndef PICKUP_STATE_MACHINE_H
#define PICKUP_STATE_MACHINE_H

#include <string>
#include <unordered_map>
#include <iostream>
#include <algorithm>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_srvs/SetBool.h>

#include <uav_ros_control/filters/Util.hpp>
#include <uav_ros_control/VisualServoStateMachineParametersConfig.h>
#include <uav_ros_control/filters/NonlinearFilters.hpp>
#include <uav_ros_control/reference/TrajectoryGenerator.hpp>

namespace uav_reference {

typedef trajectory_msgs::MultiDOFJointTrajectory Traj;
typedef trajectory_msgs::MultiDOFJointTrajectoryPoint TrajPoint;
typedef uav_ros_control::VisualServoStateMachineParametersConfig PickupParams;

enum class PickupState { OFF, ALIGNMENT, DESCENT, PICKUP_ALIGNMENT, PICKUP };

std::ostream &operator<<(std::ostream &o, const PickupState &state)
{
  switch (state) {
  case PickupState::OFF:
    o << "[OFF]";
    break;

  case PickupState::ALIGNMENT:
    o << "[ALIGNMENT]";
    break;

  case PickupState::DESCENT:
    o << "[DESCENT]";
    break;

  case PickupState::PICKUP_ALIGNMENT:
    o << "[PICKUP_ALIGNMENT]";
    break;

  case PickupState::PICKUP:
    o << "[PICKUP]";
    break;
  }
  return o;
}

class PickupStateMachine
{
public:
  explicit PickupStateMachine(ros::NodeHandle &t_nh)
    : m_handlerYawError(t_nh, "yaw_error"),
      m_handlerGlobalCentroid(t_nh, "centroid_global"),
      m_handlerLocalCentroid(t_nh, "centroid_local"), m_handlerOdometry(t_nh, "odometry"),
      m_currentState(PickupState::OFF)
  {
    initializeParameters(t_nh);
    initializeStateActions();
    initializeStateTransitions();

    m_pubPickupState = t_nh.advertise<std_msgs::Int32>("pickup_sm/state", 1);
    m_pubTrajPointGen = t_nh.advertise<Traj>("generator/trajectory", 1);
    m_pubTrajectoryPoint = t_nh.advertise<TrajPoint>("trajectory_point", 1);
    // Setup brick pickup service callback
    m_serviceBrickPickup = t_nh.advertiseService(
      "brick_pickup", &uav_reference::PickupStateMachine::brickPickupServiceCb, this);

    double rate =
      ros_util::getParamOrThrow<double>(t_nh, "visual_servo/state_machine/rate");
    m_dt = 1.0 / rate;
    m_loopTimer = t_nh.createTimer(
      ros::Duration(m_dt), &uav_reference::PickupStateMachine::loopCallback, this);
  }

private:
  inline bool isPickupActive() const { return m_currentState != PickupState::OFF; }

  inline bool isBrickDetectionValid()
  {
    return !(m_handlerGlobalCentroid.getData().x == INVALID_DISTANCE
             || m_handlerGlobalCentroid.getData().y == INVALID_DISTANCE
             || m_handlerGlobalCentroid.getData().z == INVALID_DISTANCE);
  }

  bool isTargetInThreshold(const double minX,
    const double minY,
    const double minZ,
    const double targetDistance)
  {

    // TODO: Tranasform local centroid and compensate !!11
    double tarx = fabs(m_handlerLocalCentroid.getData().x),
           tary = fabs(m_handlerLocalCentroid.getData().y),
           tarz = fabs(-m_handlerLocalCentroid.getData().z - targetDistance);

    return tarx < minX && tary < minY && tarz < minZ;
  }

  bool isUavVelcityInThreshold()
  {
    double velx = fabs(m_handlerOdometry.getData().twist.twist.linear.x),
           vely = fabs(m_handlerOdometry.getData().twist.twist.linear.y),
           velz = fabs(m_handlerOdometry.getData().twist.twist.linear.z);

    return velx < m_handlerParams->getData().min_touchdown_uav_velocity_error_xy
           && vely < m_handlerParams->getData().min_touchdown_uav_velocity_error_xy
           && velz < m_handlerParams->getData().min_touchdown_uav_velocity_error_z;
  }

  bool isYawInThreshold()
  {
    // TODO: Run VisualServo as a node
    // TODO: Add yaw added offset as a parametar
    return fabs(m_handlerYawError.getData().data)
           < m_handlerParams->getData().min_yaw_error;
  }

  bool belowPickupHeight()
  {
    return -m_handlerLocalCentroid.getData().z
           <= m_handlerParams->getData().touchdown_height;
  }

  void generatePickupTrajectory()
  {
    // This function assumes there is some sort of underlying trajectory generation
    // happening
    Traj inputTrajectory;
    inputTrajectory.header.stamp = ros::Time::now();
    inputTrajectory.points = std::vector<TrajPoint>(3);
    inputTrajectory.points[0] = m_currentTrajectoryPoint;
    inputTrajectory.points[1] =
      traj_gen::toTrajectoryPointMsg(m_currentTrajectoryPoint.transforms[0].translation.x,
        m_currentTrajectoryPoint.transforms[0].translation.y,
        m_handlerGlobalCentroid.getData().z// Go to where the brick is (globally)
          + m_handlerParams->getData().magnet_offset,// ... and add magnet offset
        m_currentTrajectoryPoint.transforms[0].rotation.x,
        m_currentTrajectoryPoint.transforms[0].rotation.y,
        m_currentTrajectoryPoint.transforms[0].rotation.z,
        m_currentTrajectoryPoint.transforms[0].rotation.w);
    inputTrajectory.points[2] = m_currentTrajectoryPoint;
    m_pubTrajPointGen.publish(inputTrajectory);
  }

  void doTheStateAction() { m_stateActionMap[m_currentState](); }

  void publishState()
  {
    std_msgs::Int32 msg;
    msg.data = static_cast<int>(m_currentState);
    m_pubPickupState.publish(msg);
  }

  void loopCallback(const ros::TimerEvent &)
  {
    doTheStateAction();
    publishState();

    const auto near_brick_x = [&]() {
      return m_handlerOdometry.getData().pose.pose.position.x
             + 0.5
                 * (m_handlerGlobalCentroid.getData().x
                     - m_handlerOdometry.getData().pose.pose.position.x);
    };

    const auto near_brick_y = [&]() {
      return m_handlerOdometry.getData().pose.pose.position.y
             + 0.5
                 * (m_handlerGlobalCentroid.getData().y
                     - m_handlerOdometry.getData().pose.pose.position.y);
    };

    const auto near_brick_yaw = [&]() {
      double uavYaw =
        util::calculateYaw(m_handlerOdometry.getData().pose.pose.orientation.x,
          m_handlerOdometry.getData().pose.pose.orientation.y,
          m_handlerOdometry.getData().pose.pose.orientation.z,
          m_handlerOdometry.getData().pose.pose.orientation.w);
      return uavYaw - 0.2 * m_handlerYawError.getData().data;
    };

    const auto height_servo_ref = [&](const double servoSetpoint) {
      return m_handlerOdometry.getData().pose.pose.position.z
             + double(m_handlerParams->getData().detection_counter) / 100.0
                 * (servoSetpoint - (-m_handlerLocalCentroid.getData().z));
    };

    // Publish some trajectory points
    switch (m_currentState) {

    case PickupState::OFF:
      // Do nothing: should be in position hold
      break;

    case PickupState::ALIGNMENT:

      m_currentTrajectoryPoint = traj_gen::toTrajectoryPointMsg(near_brick_x(),
        near_brick_y(),
        height_servo_ref(m_handlerParams->getData().brick_alignment_height),
        near_brick_yaw());
      m_pubTrajectoryPoint.publish(m_currentTrajectoryPoint);
      break;

    case PickupState::DESCENT:
      m_currentTrajectoryPoint.transforms[0].translation.x = near_brick_x();
      m_currentTrajectoryPoint.transforms[0].translation.y = near_brick_y();
      m_currentTrajectoryPoint.transforms[0].translation.z -=
        m_handlerParams->getData().descent_speed * m_dt;
      m_pubTrajectoryPoint.publish(m_currentTrajectoryPoint);
      break;

    case PickupState::PICKUP_ALIGNMENT:
      m_touchdownAlignDuration += m_dt;
      m_currentTrajectoryPoint.transforms[0].translation.x = near_brick_x();
      m_currentTrajectoryPoint.transforms[0].translation.y = near_brick_y();
      m_currentTrajectoryPoint.transforms[0].translation.z =
        height_servo_ref(m_handlerParams->getData().touchdown_height);
      m_pubTrajectoryPoint.publish(m_currentTrajectoryPoint);
      break;

    case PickupState::PICKUP:
      // Do nothing, a call to another node executes trajectory
      break;
    };
  }

  bool brickPickupServiceCb(std_srvs::SetBool::Request &request,
    std_srvs::SetBool::Response &response)
  {
    const auto is_pickup_requested = [&request]() { return request.data; };

    // Case: Pickup is already activated
    if (isPickupActive() && is_pickup_requested()) {
      ROS_WARN("PickupStateMachine: - pickup is already active");
      response.success = true;
      response.message = "Pickup is already active";
      return true;
    }

    // Case: Start pickup attempt
    if (!isPickupActive() && is_pickup_requested()) {
      ROS_WARN("PickupStateMachine - starting ALIGNMENT");
      bool success = m_stateTransitionMap[m_currentState](PickupState::ALIGNMENT);
      response.success = success;
      response.message = "Pickup state - check success field";
      return true;
    }

    // Case: Stop pickup attempt, pickup is activated
    if (!isPickupActive() && !is_pickup_requested()) {
      ROS_WARN("PickupStateMachine - pickup is already disabled");
      response.success = false;
      response.message = "Pickup is already disabled";
      return true;
    }

    // Case: Stop pickup attempt, pickup is deactivated
    if (isPickupActive() && !is_pickup_requested()) {
      ROS_FATAL("PickupStateMachin - activated OFF");
      m_currentState = PickupState::OFF;
      response.success = false;
      response.message = "Pickup state - check success field";
    }
  }

  void initializeStateTransitions()
  {
    const auto offTransition = [&](const PickupState &state) {
      // OFF -> ~something not ALIGNMENT
      if (state != PickupState::ALIGNMENT) {
        ROS_FATAL_STREAM("PickupStateMachine: Invalid transition: " << PickupState::OFF
                                                                    << " -> " << state);
        return false;
      }

      // OFF -> ALIGNMENT
      ROS_INFO_STREAM("PickupStateMachine: " << PickupState::OFF << " -> " << state);
      m_currentState = PickupState::ALIGNMENT;
      m_currentTrajectoryPoint =
        traj_gen::toTrajectoryPointMsg(m_handlerOdometry.getData().pose.pose.position.x,
          m_handlerOdometry.getData().pose.pose.position.y,
          m_handlerOdometry.getData().pose.pose.position.z,
          0);
      return true;
    };

    const auto alignmentTransition = [&](const PickupState &state) {
      // Alwayes return to off when requested
      if (!isBrickDetectionValid()) {
        ROS_WARN_STREAM("PickupStateMachine: invalid measurement "
                        << PickupState::ALIGNMENT << " -> " << PickupState::OFF);
        m_currentState = PickupState::OFF;
        return true;

      } else if (state != PickupState::DESCENT) {
        ROS_FATAL_STREAM("PickupStateMachine: Invalid transition: "
                         << PickupState::ALIGNMENT << " -> " << state);
        return false;
      }

      if (isYawInThreshold()
          && isTargetInThreshold(m_handlerParams->getData().min_error,
               m_handlerParams->getData().min_error,
               m_handlerParams->getData().min_error,
               m_handlerParams->getData().brick_alignment_height)) {
        ROS_INFO_STREAM(
          "PickupStateMachine: " << PickupState::ALIGNMENT << " -> " << state);
        m_currentState = PickupState::DESCENT;
        return true;
      }

      return false;
    };

    const auto descentTransition = [&](const PickupState &state) {
      // Alwayes return to off when requested
      if (!isBrickDetectionValid()) {
        ROS_INFO_STREAM(
          "PickupStateMachine: " << PickupState::DESCENT << " -> " << PickupState::OFF);
        m_currentState = PickupState::OFF;
        return true;

      } else if (state != PickupState::PICKUP_ALIGNMENT) {
        ROS_FATAL_STREAM("PickupStateMachine: Invalid transition: "
                         << PickupState::DESCENT << " -> " << state);
        return false;
      }


      if (belowPickupHeight()) {
        ROS_INFO_STREAM(
          "PickupStateMachine: " << PickupState::DESCENT << " -> " << state);
        m_currentState = PickupState::PICKUP_ALIGNMENT;
        m_touchdownAlignDuration = 0;
        return true;
      }

      return false;
    };

    const auto pickupAlignmentTransition = [&](const PickupState &state) {
      // Alwayes return to off when requested
      if (!isBrickDetectionValid()) {
        ROS_INFO_STREAM("PickupStateMachine: " << PickupState::PICKUP_ALIGNMENT << " -> "
                                               << PickupState::OFF);
        m_currentState = PickupState::OFF;
        return true;

      } else if (state != PickupState::PICKUP) {
        ROS_FATAL_STREAM("PickupStateMachine: Invalid transition: "
                         << PickupState::PICKUP_ALIGNMENT << " -> " << state);
        return false;
      }

      if (isUavVelcityInThreshold()
          && isTargetInThreshold(
               m_handlerParams->getData().min_touchdown_target_position_error_xy,
               m_handlerParams->getData().min_touchdown_target_position_error_xy,
               m_handlerParams->getData().min_touchdown_target_position_error_z,
               m_handlerParams->getData().touchdown_height)
          && m_touchdownAlignDuration
               >= m_handlerParams->getData().min_touchdown_align_duration) {
        ROS_INFO_STREAM(
          "PickupStateMachine: " << PickupState::PICKUP_ALIGNMENT << " -> " << state);
        m_currentState = PickupState::PICKUP;
        generatePickupTrajectory();
        // TODO: Generate trajectory here.
        return true;
      }

      return false;
    };

    const auto pickupTransition = [&](const PickupState &state) {
      // Alwayes return to off when requested
      if (state
          == PickupState::OFF) {// TODO: Add check for when pickup trajectory is finished
        ROS_INFO_STREAM("PickupStateMachine: " << m_currentState << " -> " << state);
        m_currentState = PickupState::OFF;
        return true;

      } else {
        // There are no more valid transitions
        ROS_FATAL_STREAM("PickupStateMachine: Invalid transition: " << PickupState::PICKUP
                                                                    << " -> " << state);
        return false;
      }
    };

    m_stateTransitionMap.emplace(PickupState::OFF, offTransition);
    m_stateTransitionMap.emplace(PickupState::DESCENT, descentTransition);
    m_stateTransitionMap.emplace(PickupState::ALIGNMENT, alignmentTransition);
    m_stateTransitionMap.emplace(
      PickupState::PICKUP_ALIGNMENT, pickupAlignmentTransition);
    m_stateTransitionMap.emplace(PickupState::PICKUP, pickupTransition);
  }

  void initializeStateActions()
  {
    const auto offStateAction = [&]() {
      // Do nothing
    };

    const auto alignmentStateAction = [&]() {
      m_stateTransitionMap[m_currentState](PickupState::DESCENT);
    };

    const auto descentStateAction = [&]() {
      m_stateTransitionMap[m_currentState](PickupState::PICKUP_ALIGNMENT);
    };

    const auto pickupAlignmentAction = [&]() {
      m_stateTransitionMap[m_currentState](PickupState::PICKUP);
    };

    const auto pickupStateAction = [&]() {
      m_stateTransitionMap[m_currentState](PickupState::OFF);
    };

    m_stateActionMap.emplace(PickupState::OFF, offStateAction);
    m_stateActionMap.emplace(PickupState::DESCENT, descentStateAction);
    m_stateActionMap.emplace(PickupState::ALIGNMENT, alignmentStateAction);
    m_stateActionMap.emplace(PickupState::PICKUP_ALIGNMENT, pickupAlignmentAction);
    m_stateActionMap.emplace(PickupState::PICKUP, pickupStateAction);
  }

  void initializeParameters(ros::NodeHandle &t_nh)
  {
    PickupParams paramConfig;
    paramConfig.after_touchdown_height = ros_util::getParamOrThrow<double>(
      t_nh, "visual_servo/state_machine/after_touchdown_height");
    paramConfig.ascent_speed =
      ros_util::getParamOrThrow<double>(t_nh, "visual_servo/state_machine/ascent_speed");
    paramConfig.brick_alignment_height = ros_util::getParamOrThrow<double>(
      t_nh, "visual_servo/state_machine/brick_alignment_height");
    paramConfig.descent_speed =
      ros_util::getParamOrThrow<double>(t_nh, "visual_servo/state_machine/descent_speed");
    paramConfig.detection_counter = ros_util::getParamOrThrow<double>(
      t_nh, "visual_servo/state_machine/detection_counter");
    paramConfig.disable_visual_servo_touchdown_height = ros_util::getParamOrThrow<double>(
      t_nh, "visual_servo/state_machine/disable_visual_servo_touchdown_height");
    paramConfig.magnet_offset = ros_util::getParamOrThrow<double>(
      t_nh, "visual_servo/state_machine/disable_visual_servo_touchdown_height");
    paramConfig.min_error =
      ros_util::getParamOrThrow<double>(t_nh, "visual_servo/state_machine/min_error");
    paramConfig.min_touchdown_align_duration = ros_util::getParamOrThrow<double>(
      t_nh, "visual_servo/state_machine/min_touchdown_align_duration");
    paramConfig.min_touchdown_target_position_error_xy =
      ros_util::getParamOrThrow<double>(
        t_nh, "visual_servo/state_machine/min_touchdown_target_position_error_xy");
    paramConfig.min_touchdown_target_position_error_z = ros_util::getParamOrThrow<double>(
      t_nh, "visual_servo/state_machine/min_touchdown_target_position_error_z");
    paramConfig.min_touchdown_uav_velocity_error_xy = ros_util::getParamOrThrow<double>(
      t_nh, "visual_servo/state_machine/min_touchdown_uav_velocity_error_xy");
    paramConfig.min_touchdown_uav_velocity_error_z = ros_util::getParamOrThrow<double>(
      t_nh, "visual_servo/state_machine/min_touchdown_uav_velocity_error_z");
    paramConfig.min_yaw_error =
      ros_util::getParamOrThrow<double>(t_nh, "visual_servo/state_machine/min_yaw_error");
    paramConfig.touchdown_height = ros_util::getParamOrThrow<double>(
      t_nh, "visual_servo/state_machine/touchdown_height");
    paramConfig.touchdown_speed = ros_util::getParamOrThrow<double>(
      t_nh, "visual_servo/state_machine/touchdown_speed");

    m_handlerParams.reset(
      new ros_util::ParamHandler<PickupParams>(paramConfig, "pickup_config"));
  }

  static constexpr int INVALID_DISTANCE = -1;

  ros::Publisher m_pubTrajectoryPoint, m_pubTrajPointGen, m_pubPickupState;
  TrajPoint m_currentTrajectoryPoint;
  double m_dt = 0, m_touchdownAlignDuration = 0;

  ros_util::TopicHandler<std_msgs::Float32> m_handlerYawError;
  ros_util::TopicHandler<geometry_msgs::Vector3> m_handlerGlobalCentroid,
    m_handlerLocalCentroid;
  ros_util::TopicHandler<nav_msgs::Odometry> m_handlerOdometry;
  std::unique_ptr<ros_util::ParamHandler<PickupParams>> m_handlerParams;

  PickupState m_currentState;
  std::unordered_map<PickupState, std::function<void()>, ros_util::EnumClassHash>
    m_stateActionMap;
  std::unordered_map<PickupState,
    std::function<bool(const PickupState &)>,
    ros_util::EnumClassHash>
    m_stateTransitionMap;
  ros::ServiceServer m_serviceBrickPickup;
  ros::Timer m_loopTimer;
};

}// namespace uav_reference

#endif// PICKUP_STATE_MACHINE