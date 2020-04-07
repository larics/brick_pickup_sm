#ifndef VISUAL_SERVO_STATE_MACHINE_H
#define VISUAL_SERVO_STATE_MACHINE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <dynamic_reconfigure/server.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <uav_ros_control/VisualServoStateMachineParametersConfig.h>
#include <uav_ros_control_msgs/VisualServoProcessValues.h>
#include <uav_ros_control/filters/NonlinearFilters.hpp>
#include <uav_ros_control/reference/PickupStates.hpp>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

namespace uav_reference {

typedef uav_ros_control::VisualServoStateMachineParametersConfig vssm_param_t;
#define VSSM_DYN_RECONF "brick_config/vs_state_machine"
#define PARAM_MIN_ERROR "visual_servo/state_machine/min_error"
#define PARAM_MIN_TD_TAR_ERROR_Z \
  "visual_servo/state_machine/min_touchdown_target_position_error_z"
#define PARAM_MIN_TD_UAV_VEL_ERROR_Z \
  "visual_servo/state_machine/min_touchdown_uav_velocity_error_z"
#define PARAM_MIN_TD_TAR_ERROR_XY \
  "visual_servo/state_machine/min_touchdown_target_position_error_xy"
#define PARAM_MIN_TD_UAV_VEL_ERROR_XY \
  "visual_servo/state_machine/min_touchdown_uav_velocity_error_xy"
#define PARAM_MIN_TD_ALIGN_DURATION \
  "visual_servo/state_machine/min_touchdown_align_duration"
#define PARAM_MIN_YAW_ERROR "visual_servo/state_machine/min_yaw_error"
#define PARAM_VS_HEIGHT_DISABLE \
  "visual_servo/state_machine/disable_visual_servo_touchdown_height"
#define PARAM_TOUCHDOWN_HEIGHT "visual_servo/state_machine/touchdown_height"
#define PARAM_MAGNET_OFFSET "visual_servo/state_machine/magnet_offset"
#define PARAM_TOUCHDOWN_SPEED "visual_servo/state_machine/touchdown_speed"
#define PARAM_RATE "visual_servo/state_machine/rate"
#define PARAM_DESCENT_SPEED "visual_servo/state_machine/descent_speed"
#define PARAM_ASCENT_SPEED "visual_servo/state_machine/ascent_speed"
#define PARAM_DET_COUNTER "visual_servo/state_machine/detection_counter"
#define PARAM_AFTER_TD_HEIGHT "visual_servo/state_machine/after_touchdown_height"
#define PARAM_BRICK_ALIGN_HEIGHT "visual_servo/state_machine/brick_alignment_height"
#define INVALID_DISTANCE -1

using namespace pickup_states;
class VisualServoStateMachine
{

public:
  VisualServoStateMachine(ros::NodeHandle &nh)
  {
    initializeParameters(nh);
    // Define Publishers
    _pubVisualServoFeed = nh.advertise<uav_ros_control_msgs::VisualServoProcessValues>(
      "visual_servo/process_value", 1);
    _pubVssmState = nh.advertise<std_msgs::Int32>("visual_servo_sm/state", 1);
    _pubRelativeDistance_global =
      nh.advertise<std_msgs::Float32>("visual_servo_sm/distance/global", 1);
    _pubRelativeDistance_local =
      nh.advertise<std_msgs::Float32>("visual_servo_sm/distance/local", 1);
    _pubVelError =
      nh.advertise<geometry_msgs::Vector3>("visual_servo_sm/velocity_error", 1);
    _pubTargetError =
      nh.advertise<geometry_msgs::Vector3>("visual_servo_sm/pos_error", 1);
    _pubLoopStatus = nh.advertise<std_msgs::Int32>("visual_servo_sm/loop_status", 1);

    // Define Subscribers
    _subOdom =
      nh.subscribe("odometry", 1, &uav_reference::VisualServoStateMachine::odomCb, this);
    // Note from past Lovro : This is changed to debug/yaw_error because that holds actual
    // information about the yaw_error
    // TODO: Change this to make more sense
    _subYawError = nh.subscribe(
      "debug/yaw_error", 1, &uav_reference::VisualServoStateMachine::yawErrorCb, this);
    _subNContours = nh.subscribe(
      "n_contours", 1, &uav_reference::VisualServoStateMachine::nContoursCb, this);
    _subPatchCentroid_global = nh.subscribe("global_centroid_point",
      1,
      &uav_reference::VisualServoStateMachine::globalCentroidPointCb,
      this);
    _subPatchCentroid_local = nh.subscribe("local_centroid_point",
      1,
      &uav_reference::VisualServoStateMachine::localCentroidPointCb,
      this);
    _subTrajectory = nh.subscribe(
      "carrot/trajectory", 1, &uav_reference::VisualServoStateMachine::trajCb, this);

    // Setup dynamic reconfigure server
    vssm_param_t vssmConfig;
    setVSSMParameters(vssmConfig);
    _vssmConfigServer.updateConfig(vssmConfig);
    _vssmParamCallback =
      boost::bind(&uav_reference::VisualServoStateMachine::vssmParamCb, this, _1, _2);
    _vssmConfigServer.setCallback(_vssmParamCallback);

    // Setup brick pickup service callback
    _serviceBrickPickup = nh.advertiseService("brick_pickup/local",
      &uav_reference::VisualServoStateMachine::brickPickupServiceCb,
      this);

    // Initialize visual servo client caller
    _vsClienCaller =
      nh.serviceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
        "visual_servo");
  }

  ~VisualServoStateMachine() {}

  void trajCb(const trajectory_msgs::MultiDOFJointTrajectoryPoint &point)
  {
    _trajPoint = point;
  }

  void nContoursCb(const std_msgs::Int32ConstPtr &msg)
  {
    _nContours = msg->data;
    if (msg->data == 0 && _currentState != LocalPickupState::TOUCHDOWN
        && _currentState != LocalPickupState::OFF) {
      ROS_FATAL("VSSM - patch count is 0.");
      turnOffVisualServo();
    }
    _timeLastContour = ros::Time::now().toSec();
  }

  void localCentroidPointCb(const geometry_msgs::Vector3 &msg)
  {
    _localCentroid = msg;
    if (msg.z == INVALID_DISTANCE) {
      _relativeBrickDistance_local = INVALID_DISTANCE;
    } else {
      _relativeBrickDistance_local =
        -msg.z;// Minus sign here because Centroid is wrt. the UAV base frame
    }
    std_msgs::Float32 newMessage;
    newMessage.data = _relativeBrickDistance_local;
    _pubRelativeDistance_local.publish(newMessage);
    _timeLastCentroidLocal = ros::Time::now().toSec();
  }

  void globalCentroidPointCb(const geometry_msgs::Vector3 &msg)
  {
    static constexpr double MIN_CENTROID_DISTANCE = 1;

    // Check if global centroid is same as previous centroid
    if (_currentState != LocalPickupState::OFF// Not in OFF state
        && _currentState != LocalPickupState::TOUCHDOWN// .. and Not in TOUCHDOWN state
        && isRelativeDistanceValid(msg.z)// .. and relative distance is valid
        && sqrt(pow(_globalCentroid.x - msg.x, 2) + pow(_globalCentroid.y - msg.y, 2)
                + pow(_globalCentroid.z - msg.z, 2))
             > MIN_CENTROID_DISTANCE)// .. and not close to precious centroid
    {
      ROS_FATAL("VSSM - new centroid [%.2f, %.2f, %.2f] is not old [%.2f, %.2f, %.2f]",
        msg.x,
        msg.y,
        msg.z,
        _globalCentroid.x,
        _globalCentroid.y,
        _globalCentroid.z);
      turnOffVisualServo();
    }

    _globalCentroid = msg;
    // TODO: provjeriti sa relativnom udaljenoscu
    if (msg.z == INVALID_DISTANCE) {
      _relativeBrickDistance_global = INVALID_DISTANCE;
    } else {
      _relativeBrickDistance_global = msg.z;
    }

    std_msgs::Float32 newMessage;
    newMessage.data = _relativeBrickDistance_global;
    _pubRelativeDistance_global.publish(newMessage);
    _timeLastCentroidGlobal = ros::Time::now().toSec();
  }

  bool healthyNumberOfPublishers()
  {
    ROS_FATAL_COND((!_subNContours.getNumPublishers()) > 0,
      "VSSM - 'n_contours' topic publisher missing");
    ROS_FATAL_COND(
      !_subOdom.getNumPublishers() > 0, "VSSM - 'odometry' topic publisher missing");
    ROS_FATAL_COND(!_subPatchCentroid_global.getNumPublishers() > 0,
      "VSSM - 'centroid_global' topic publisher missing");
    ROS_FATAL_COND(!_subPatchCentroid_local.getNumPublishers() > 0,
      "VSSM - 'centroid_local' topic publisher missing");
    ROS_FATAL_COND(
      !_subYawError.getNumPublishers() > 0, "VSSM - 'yaw_error' topic publisher missing");

    return _subNContours.getNumPublishers() > 0 && _subOdom.getNumPublishers() > 0
           && _subPatchCentroid_global.getNumPublishers() > 0
           && _subPatchCentroid_local.getNumPublishers() > 0
           && _subYawError.getNumPublishers() > 0;
  }

  bool brickPickupServiceCb(std_srvs::SetBool::Request &request,
    std_srvs::SetBool::Response &response)
  {
    if (!request.data || stateMachineDisableConditions()
        || !healthyNumberOfPublishers()) {
      if (!healthyNumberOfPublishers())
        ROS_FATAL_THROTTLE(
          THROTTLE_TIME, "VSSM::brickPickupServiceCb - check connected publishers.");

      if (!request.data)
        ROS_FATAL_THROTTLE(THROTTLE_TIME,
          "VSSM::brickPickupServiceCb - brick pickup deactivation requested.");

      if (_nContours == 0)
        ROS_FATAL_THROTTLE(
          THROTTLE_TIME, "VSSM::brickPickupServiceCb - no contours found.");

      if (!isRelativeDistanceValid(_relativeBrickDistance_local)
          || !isRelativeDistanceValid(_relativeBrickDistance_global))
        ROS_FATAL_THROTTLE(
          THROTTLE_TIME, "VSSM::brickPIckupServiceCb - distances invalid");

      turnOffVisualServo();
      _brickPickupActivated = false;
      response.success = false;
      response.message = "Visual servo and brick pickup deactivated";
      return true;
    }

    // Check if brick pickup is already activated.
    if (_brickPickupActivated) {
      ROS_FATAL("VSSM::brickPickupServiceCb - brick pickup is already active.");
      response.success = false;
      response.message = "Brick pickup is already active";
      return true;
    }

    // Try calling visual servo
    ros::spinOnce();
    publishVisualServoSetpoint(1.0 / _rate);

    std_srvs::SetBool::Request req;
    std_srvs::SetBool::Response resp;
    req.data = true;
    if (!_vsClienCaller.call(req, resp)) {
      ROS_FATAL("VSSM::brickPickupServiceCb - calling visual servo failed.");
      response.success = false;
      response.message = "Service caller for visual servo failed.";
      _currentState = LocalPickupState::OFF;
      return true;
    }

    if (resp.success) {
      // Visual servo successfully activated
      ROS_INFO("VSSM::brickPickupServiceCb() - brick pickup activated.");
      response.success = true;
      response.message = "Visual servo enabled - brick pickup activated.";
      _brickPickupActivated = true;
      return true;
    }

    ROS_WARN("VSSM::brickPickupServiceCb - unable to activate brick pickup.");
    response.success = false;
    response.message = "Visual servo failed to start - brick pickup inactive.";
    _brickPickupActivated = false;

    return true;
  }

  void vssmParamCb(vssm_param_t &configMsg, uint32_t level)
  {
    ROS_WARN("VisualServoStateMachine::vssmParamCb()");
    _minTargetError = configMsg.min_error;
    _minYawError = configMsg.min_yaw_error;
    _touchdownHeight = configMsg.touchdown_height;
    _magnetOffset = configMsg.magnet_offset;
    _touchdownSpeed = configMsg.touchdown_speed;
    _descentSpeed = configMsg.descent_speed;
    _afterTouchdownHeight = configMsg.after_touchdown_height;
    _descentCounterMax = configMsg.detection_counter;
    _ascentSpeed = configMsg.ascent_speed;
    _minTouchdownTargetPositionError_z = configMsg.min_touchdown_target_position_error_z;
    _minTouchdownUavVelocityError_z = configMsg.min_touchdown_uav_velocity_error_z;
    _minTouchdownTargetPositionError_xy =
      configMsg.min_touchdown_target_position_error_xy;
    _minTouchdownUavVelocityError_xy = configMsg.min_touchdown_uav_velocity_error_xy;
    _minTouchdownAlignDuration = configMsg.min_touchdown_align_duration;
    _visualServoDisableHeight = configMsg.disable_visual_servo_touchdown_height;
    _brickAlignHeight = configMsg.brick_alignment_height;
  }

  void setVSSMParameters(vssm_param_t &config)
  {
    config.min_error = _minTargetError;
    config.min_yaw_error = _minYawError;
    config.magnet_offset = _magnetOffset;
    config.touchdown_speed = _touchdownSpeed;
    config.touchdown_height = _touchdownHeight;
    config.descent_speed = _descentSpeed;
    config.after_touchdown_height = _afterTouchdownHeight;
    config.detection_counter = _descentCounterMax;
    config.ascent_speed = _ascentSpeed;
    config.min_touchdown_target_position_error_z = _minTouchdownTargetPositionError_z;
    config.min_touchdown_uav_velocity_error_z = _minTouchdownUavVelocityError_z;
    config.min_touchdown_target_position_error_xy = _minTouchdownTargetPositionError_xy;
    config.min_touchdown_uav_velocity_error_xy = _minTouchdownUavVelocityError_xy;
    config.min_touchdown_align_duration = _minTouchdownAlignDuration;
    config.disable_visual_servo_touchdown_height = _visualServoDisableHeight;
    config.brick_alignment_height = _brickAlignHeight;
  }

  void initializeParameters(ros::NodeHandle &nh)
  {
    ROS_INFO("VisualServoStateMachine::initializeParameters()");
    bool initialized =
      nh.getParam(PARAM_MIN_YAW_ERROR, _minYawError)
      && nh.getParam(PARAM_MIN_TD_TAR_ERROR_XY, _minTouchdownTargetPositionError_xy)
      && nh.getParam(PARAM_MIN_TD_UAV_VEL_ERROR_XY, _minTouchdownUavVelocityError_xy)
      && nh.getParam(PARAM_MIN_TD_TAR_ERROR_Z, _minTouchdownTargetPositionError_z)
      && nh.getParam(PARAM_MIN_TD_UAV_VEL_ERROR_Z, _minTouchdownUavVelocityError_z)
      && nh.getParam(PARAM_VS_HEIGHT_DISABLE, _visualServoDisableHeight)
      && nh.getParam(PARAM_MIN_TD_ALIGN_DURATION, _minTouchdownAlignDuration)
      && nh.getParam(PARAM_BRICK_ALIGN_HEIGHT, _brickAlignHeight)
      && nh.getParam(PARAM_MIN_ERROR, _minTargetError)
      && nh.getParam(PARAM_TOUCHDOWN_HEIGHT, _touchdownHeight)
      && nh.getParam(PARAM_TOUCHDOWN_SPEED, _touchdownSpeed)
      && nh.getParam(PARAM_MAGNET_OFFSET, _magnetOffset)
      && nh.getParam(PARAM_DESCENT_SPEED, _descentSpeed)
      && nh.getParam(PARAM_ASCENT_SPEED, _ascentSpeed) && nh.getParam(PARAM_RATE, _rate)
      && nh.getParam(PARAM_AFTER_TD_HEIGHT, _afterTouchdownHeight)
      && nh.getParam(PARAM_DET_COUNTER, _descentCounterMax);

    _afterTouchdownHeight_GPS = _afterTouchdownHeight;
    ROS_INFO("Node rate: %.2f", _rate);
    ROS_INFO("Minimum yaw error: %.2f", _minYawError);
    ROS_INFO("Brick alignment height: %.2f", _brickAlignHeight);
    ROS_INFO("Min target error %.2f", _minTargetError);
    ROS_INFO("Touchdown position target error [%.2f, %.2f, %.2f]",
      _minTouchdownTargetPositionError_xy,
      _minTouchdownTargetPositionError_xy,
      _minTouchdownTargetPositionError_z);
    ROS_INFO("Touchdown uav velocity error [%.2f, %.2f, %.2f]",
      _minTouchdownUavVelocityError_xy,
      _minTouchdownUavVelocityError_xy,
      _minTouchdownUavVelocityError_z);
    ROS_INFO("Min touchdown alignment duration %.2f", _minTouchdownAlignDuration);
    ROS_INFO("Visual servo disable height %.2f", _visualServoDisableHeight);
    ROS_INFO("Descent speed: %.2f", _descentSpeed);
    ROS_INFO("Touchdown height: %.2f", _touchdownHeight);
    ROS_INFO("Touchdown speed: %.2f", _touchdownSpeed);
    ROS_INFO("Magnet offset: %.2f", _magnetOffset);
    ROS_INFO("After touchdown height: %.2f", _afterTouchdownHeight);
    ROS_INFO("Detection counter: %d", _descentTransitionCounter);
    if (!initialized) {
      ROS_FATAL(
        "VisualServoStateMachine::initializeParameters() - failed to initialize "
        "parameters");
      throw std::runtime_error(
        "VisualServoStateMachine parameters not properly initialized.");
    }
  }

  void turnOffVisualServo()
  {
    // Attempt to turn off visual servo
    std_srvs::SetBool::Request req;
    std_srvs::SetBool::Response resp;
    req.data = false;
    if (!_vsClienCaller.call(req, resp)) {
      ROS_FATAL("VSSM::updateStatus - calling visual servo failed.");
      return;
    }

    if (!resp.success) {
      ROS_INFO_THROTTLE(
        THROTTLE_TIME, "VSSM::updateStatus - visual servo successfully deactivated");
      // Visual servo successfully activated
      _currentState = LocalPickupState::OFF;
      ROS_INFO_THROTTLE(THROTTLE_TIME, "VSSM::updateStatus - OFF state activated. ");
      _brickPickupActivated = false;
      ROS_INFO_THROTTLE(THROTTLE_TIME, "VSSM::updateStatus - Brick pickup finished.");
      return;
    } else {
      // Visual servo is still active here...
      ROS_FATAL(
        "VSSM::updateStatus - Touchdown finished but unable to deactivate visual servo.");
    }
  }

  bool stateMachineDisableConditions()
  {
    return !subscribedTopicsActive()
           || !isRelativeDistanceValid(_relativeBrickDistance_local)
           || !isRelativeDistanceValid(_relativeBrickDistance_global) || _nContours == 0;
  }

  void updateState()
  {
    if (_currentState != LocalPickupState::OFF && !_brickPickupActivated
        ||// If visual servo is
        _currentState == LocalPickupState::DESCENT && stateMachineDisableConditions()
        || _currentState == LocalPickupState::BRICK_ALIGNMENT
             && stateMachineDisableConditions()
        || _currentState == LocalPickupState::TOUCHDOWN_ALIGNMENT
             && stateMachineDisableConditions()) {
      // deactivate state machine
      ROS_WARN_THROTTLE(THROTTLE_TIME, "VSSM::updateStatus - Visual servo is inactive.");
      _currentState = LocalPickupState::OFF;
      _brickPickupActivated = false;
      turnOffVisualServo();
      ROS_WARN_THROTTLE(THROTTLE_TIME, "VSSM::updateStatus - OFF State activated.");
      return;
    }

    // If brick pickup is activate start brick alignment first
    if (_currentState == LocalPickupState::OFF && _brickPickupActivated
        && isRelativeDistanceValid(_relativeBrickDistance_local)) {
      ROS_INFO("VSSM::updateStatus - Brick pickup requested");
      _currHeightReference = _currOdom.pose.pose.position.z;
      _descentTransitionCounter = 0;
      _currentState = LocalPickupState::BRICK_ALIGNMENT;
      ROS_INFO("VSSM::updateStatus - BRICK_ALIGNMENT state activated with height: %2f.",
        _currHeightReference);
      return;
    }

    // Update the transition counter
    if (_currentState == LocalPickupState::BRICK_ALIGNMENT
        && isTargetInThreshold(
             _minTargetError, _minTargetError, _minTargetError, _brickAlignHeight)
        && fabs(_currYawError) < _minYawError) {
      _descentTransitionCounter++;
    }

    // If brick alignemnt is activated and target error is withing range start descent
    if (_currentState == LocalPickupState::BRICK_ALIGNMENT
        && _descentTransitionCounter > _descentCounterMax) {
      _currentState = LocalPickupState::DESCENT;
      _relativeBrickDistanceGlobal_lastValid = _relativeBrickDistance_global;
      ROS_INFO("VSSM::updateStatus - DESCENT state activated");
      return;
    }

    // When brick alignment passes touchdown height, start alignmennt.
    if (_currentState == LocalPickupState::DESCENT
        && isRelativeDistanceValid(_relativeBrickDistance_local)
        && _relativeBrickDistance_local <= _touchdownHeight) {
      _afterTouchdownHeight_GPS =
        _currOdom.pose.pose.position.z
        + (_afterTouchdownHeight - _relativeBrickDistance_local);
      _currentState = LocalPickupState::TOUCHDOWN_ALIGNMENT;
      _currHeightReference = _currOdom.pose.pose.position.z;
      _touchdownAlignDuration = 0.1;
      ROS_INFO("VSSM::updateStatus - TOUCHDOWN_ALIGNMENT state activated");
    }

    // if height is below touchdown treshold start touchdown
    bool enabled = isUavVelcityInThreshold();
    enabled = isTargetInThreshold(_minTouchdownTargetPositionError_xy,
                _minTouchdownTargetPositionError_xy,
                _minTouchdownTargetPositionError_z,
                _touchdownHeight)
              & enabled;
    if (_currentState == LocalPickupState::TOUCHDOWN_ALIGNMENT
        && isRelativeDistanceValid(_relativeBrickDistance_local) && enabled
        && _touchdownAlignDuration >= _minTouchdownAlignDuration) {
      _currentState = LocalPickupState::TOUCHDOWN;
      _touchdownTime = 0;
      _touchdownDelta =
        _touchdownHeight// _relativeBrickDistance_local
                        //- fabs(_currHeightReference - _currOdom.pose.pose.position.z) //
                        //Take into account position tracking error
        - _magnetOffset;// Take into account magnet offset
      _relativeBrickDistanceGlobal_lastValid = _relativeBrickDistance_global;
      _touchdownDuration = _touchdownDelta / _touchdownSpeed;
      ROS_INFO(
        "VSSM::UpdateStatus - TOUCHDOWN state activated - [delta, duration] = [%.4f, "
        "%.4f]",
        _touchdownDelta,
        _touchdownDuration);
      return;
    }

    // If touchdown time is exceeded, touchdown state is considered finished
    if (_currentState == LocalPickupState::TOUCHDOWN
        && _currHeightReference >= _afterTouchdownHeight_GPS
        && _touchdownTime > _touchdownDuration) {
      ROS_INFO("VSSM::updateStatus - Touchdown duration finished.");
      turnOffVisualServo();
    }
  }

  bool isUavVelcityInThreshold()
  {
    double velx = fabs(_currOdom.twist.twist.linear.x),
           vely = fabs(_currOdom.twist.twist.linear.y),
           velz = fabs(_currOdom.twist.twist.linear.z);

    geometry_msgs::Vector3 msg;
    msg.x = velx;
    msg.y = vely;
    msg.z = velz;
    _pubVelError.publish(msg);

    return velx < _minTouchdownUavVelocityError_xy
           && vely < _minTouchdownUavVelocityError_xy
           && velz < _minTouchdownUavVelocityError_z;
  }

  bool isTargetInThreshold(const double minX,
    const double minY,
    const double minZ,
    const double targetDistance)
  {
    double tarx = fabs(_localCentroid.x), tary = fabs(_localCentroid.y),
           tarz = fabs(_relativeBrickDistance_local - targetDistance);

    geometry_msgs::Vector3 msg;
    msg.x = tarx;
    msg.y = tary;
    msg.z = tarz;
    _pubTargetError.publish(msg);

    return tarx < minX && tary < minY && tarz < minZ;
  }

  bool isRelativeDistanceValid(const double checkDistance)
  {
    return !isnan(checkDistance) && checkDistance != INVALID_DISTANCE;
  }

  bool subscribedTopicsActive()
  {
    double currentTime = ros::Time::now().toSec();
    double dt_odom = currentTime - _timeLastOdometry;
    double dt_contour = currentTime - _timeLastContour;
    double dt_centGlobal = currentTime - _timeLastCentroidGlobal;
    double dt_centLocal = currentTime - _timeLastCentroidLocal;
    double dt_yaw = currentTime - _timeLastYawError;

    static constexpr double MAX_DT = 0.5;
    ROS_FATAL_COND(dt_odom > MAX_DT, "VSSM - odometry timeout reached.");
    ROS_FATAL_COND(dt_contour > MAX_DT, "VSSM - contour timeout reached.");
    ROS_FATAL_COND(dt_centGlobal > MAX_DT, "VSSM - centroid global timeout reached.");
    ROS_FATAL_COND(dt_centLocal > MAX_DT, "VSSM - centroid local timeout reached.");
    ROS_FATAL_COND(dt_yaw > MAX_DT, "VSSM - yaw error timeout reached.");

    return dt_odom < MAX_DT && dt_contour < MAX_DT && dt_centGlobal < MAX_DT
           && dt_centLocal < MAX_DT && dt_yaw < MAX_DT;
  }

  void publishVisualServoSetpoint(double dt)
  {
    // Set VisualServoFeed to odometry
    _currVisualServoFeed.x = _currOdom.pose.pose.position.x;
    _currVisualServoFeed.y = _currOdom.pose.pose.position.y;
    _currVisualServoFeed.z = _currOdom.pose.pose.position.z;
    _currVisualServoFeed.yaw = util::calculateYaw(_currOdom.pose.pose.orientation.x,
      _currOdom.pose.pose.orientation.y,
      _currOdom.pose.pose.orientation.z,
      _currOdom.pose.pose.orientation.w);

    switch (_currentState) {
    case LocalPickupState::OFF:
      if (!_trajPoint.transforms.empty()) {
        _currVisualServoFeed.x = _trajPoint.transforms.front().translation.x;
        _currVisualServoFeed.y = _trajPoint.transforms.front().translation.y;
      }
      break;

    case LocalPickupState::BRICK_ALIGNMENT:
      _currVisualServoFeed.x = _trajPoint.transforms.front().translation.x;
      _currVisualServoFeed.y = _trajPoint.transforms.front().translation.y;
      _currVisualServoFeed.z = _currOdom.pose.pose.position.z
                               + double(_descentCounterMax) / 100.0
                                   * (_brickAlignHeight - _relativeBrickDistance_local);
      _currHeightReference = _currVisualServoFeed.z;
      break;

    case LocalPickupState::DESCENT:
      if (_currHeightReference < _relativeBrickDistanceGlobal_lastValid) {
        _currVisualServoFeed.z = _relativeBrickDistanceGlobal_lastValid;
      } else {
        _currVisualServoFeed.z = _currHeightReference - _descentSpeed * dt;
      }
      _currHeightReference = _currVisualServoFeed.z;
      _currVisualServoFeed.yaw = 0;
      break;

    case LocalPickupState::TOUCHDOWN_ALIGNMENT:
      _currVisualServoFeed.z = _currOdom.pose.pose.position.z
                               + double(_descentCounterMax) / 100.0
                                   * (_touchdownHeight - _relativeBrickDistance_local);
      _currHeightReference = _currVisualServoFeed.z;
      _currVisualServoFeed.yaw = 0;
      _touchdownAlignDuration += dt;
      break;

    case LocalPickupState::TOUCHDOWN:
      if (_relativeBrickDistance_local < _visualServoDisableHeight) {
        _currVisualServoFeed.x = 0;
        _currVisualServoFeed.y = 0;
      }

      if (_touchdownTime < _touchdownDuration) {

        if (_currHeightReference < _relativeBrickDistanceGlobal_lastValid) {
          _currVisualServoFeed.z = _relativeBrickDistanceGlobal_lastValid;
        } else {
          _currVisualServoFeed.z = _currHeightReference - _touchdownSpeed * dt;
        }

      } else {
        _currVisualServoFeed.z = _currHeightReference + _ascentSpeed * dt;
      }
      _currHeightReference = _currVisualServoFeed.z;
      _currVisualServoFeed.yaw = 0;
      _touchdownTime += dt;
      break;
    }

    _currVisualServoFeed.header.stamp = ros::Time::now();
    _pubVisualServoFeed.publish(_currVisualServoFeed);

    // Publish currrent state
    std_msgs::Int32 stateMsg;
    stateMsg.data = static_cast<int>(_currentState);
    _pubVssmState.publish(stateMsg);
  }

  void odomCb(const nav_msgs::OdometryConstPtr &msg)
  {
    _currOdom = *msg;
    _timeLastOdometry = ros::Time::now().toSec();
  }

  void yawErrorCb(const std_msgs::Float32ConstPtr &msg)
  {
    _currYawError = msg->data;
    _timeLastYawError = ros::Time::now().toSec();
  }

  void run()
  {
    ros::Rate loopRate(_rate);
    double dt = 1.0 / _rate;
    while (ros::ok()) {
      ros::spinOnce();
      std_msgs::Int32 testMsg;
      testMsg.data = 1;
      _pubLoopStatus.publish(testMsg);

      updateState();
      publishVisualServoSetpoint(dt);

      if (_currentState == LocalPickupState::DESCENT
          && !isRelativeDistanceValid(_relativeBrickDistance_local))
        ROS_FATAL("*** FATAL - BLIND DESCENT ***");// TODO: :) (?)

      loopRate.sleep();
    }
  }

private:
  static constexpr double THROTTLE_TIME = 3.0;
  double _rate = 50;

  /* Service brick pickup */
  ros::ServiceServer _serviceBrickPickup;
  bool _brickPickupActivated = false;
  LocalPickupState _currentState = LocalPickupState::OFF;

  /* Client for calling visual servo */
  ros::ServiceClient _vsClienCaller;

  /* Offset subscriber and publisher */
  ros::Publisher _pubVssmState;

  /* Pose publisher */
  ros::Publisher _pubVisualServoFeed;
  ros::Publisher _pubRelativeDistance_global;
  ros::Publisher _pubRelativeDistance_local;
  ros::Publisher _pubVelError, _pubTargetError;
  ros::Publisher _pubLoopStatus;
  uav_ros_control_msgs::VisualServoProcessValues _currVisualServoFeed;

  /* Odometry subscriber */
  ros::Subscriber _subOdom;
  nav_msgs::Odometry _currOdom;

  ros::Subscriber _subTrajectory;
  trajectory_msgs::MultiDOFJointTrajectoryPoint _trajPoint;

  /* Yaw error subscriber */
  ros::Subscriber _subYawError;
  double _currYawError = 1e5, _currUavVelError = 1e5;
  double _minYawError, _minTargetError, _minTouchdownTargetPositionError_xy,
    _minTouchdownUavVelocityError_xy, _minTouchdownTargetPositionError_z,
    _minTouchdownUavVelocityError_z, _minTouchdownAlignDuration, _brickAlignHeight;
  ros::Subscriber _subPatchCentroid_global, _subPatchCentroid_local;
  geometry_msgs::Vector3 _globalCentroid, _localCentroid;
  double _relativeBrickDistance_global = INVALID_DISTANCE,
         _relativeBrickDistance_local = INVALID_DISTANCE,
         _relativeBrickDistanceGlobal_lastValid = INVALID_DISTANCE;

  /* Contour subscriber */
  ros::Subscriber _subNContours;
  int _nContours;

  /* Touchdown mode parameters */
  double _touchdownHeight, _touchdownDelta = 0, _magnetOffset, _touchdownDuration = 0,
                           _touchdownAlignDuration = 0, _touchdownTime,
                           _descentCounterMax, _touchdownSpeed, _visualServoDisableHeight;
  double _currHeightReference, _descentSpeed, _ascentSpeed, _afterTouchdownHeight,
    _afterTouchdownHeight_GPS;
  int _descentTransitionCounter = 0;

  double _timeLastContour = 0, _timeLastOdometry = 0, _timeLastCentroidGlobal = 0,
         _timeLastCentroidLocal = 0, _timeLastYawError = 0;

  /* Define Dynamic Reconfigure parameters */
  boost::recursive_mutex _vssmConfigMutex;
  dynamic_reconfigure::Server<vssm_param_t> _vssmConfigServer{ _vssmConfigMutex,
    ros::NodeHandle(VSSM_DYN_RECONF) };
  dynamic_reconfigure::Server<vssm_param_t>::CallbackType _vssmParamCallback;
};
}// namespace uav_reference

#endif /* VISUAL_SERVO_STATE_MACHINE_H */
