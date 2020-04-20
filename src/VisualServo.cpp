//
// Created by robert on 20. 09. 2019..
//

#include <brick_pickup_sm/VisualServo.hpp>
#include <cmath>
#include <tf/LinearMath/Transform.h>
#include <uav_ros_control/filters/NonlinearFilters.hpp>

namespace uav_reference {

VisualServo::VisualServo(ros::NodeHandle &nh)
{
  initializeParameters(nh);
  const auto initialSleepTime = 2.0;
  ros::Duration(initialSleepTime).sleep();

  // Define Publishers
  _pubXError = nh.advertise<std_msgs::Float32>("visual_servo/x_error", 1);
  _pubYError = nh.advertise<std_msgs::Float32>("visual_servo/y_error", 1);
  _pubIsEnabledTopic = nh.advertise<std_msgs::Bool>("visual_servo/is_enabled", 1);
  _pubMoveLeft = nh.advertise<std_msgs::Float32>("move_left", 1);
  _pubChangeYaw = nh.advertise<std_msgs::Float32>("change_yaw", 1);
  _pubMoveForward = nh.advertise<std_msgs::Float32>("move_forward", 1);
  _pubUavYawDebug = nh.advertise<std_msgs::Float32>("debug/uav_yaw", 1);
  _pubYawErrorDebug = nh.advertise<std_msgs::Float32>("debug/yaw_error", 1);
  _pubChangeYawDebug = nh.advertise<std_msgs::Float32>(
    "debug/yaw_change", 1);// Advertised again for user friendliness
  _pubUavRollDebug = nh.advertise<std_msgs::Float32>("debug/uav_roll", 1);
  _pubUavPitchDebug = nh.advertise<std_msgs::Float32>("debug/uav_pitch", 1);
  _pubNewSetpoint = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
    "position_hold/trajectory", 1);
  _pubTransformedTarget =
    nh.advertise<geometry_msgs::Vector3>("visual_servo/centroid/transformed", 1);
  _pubTransformedTarget_local =
    nh.advertise<geometry_msgs::Vector3>("visual_servo/centroid/transformed_local", 1);
  _pubTransformedTargetComp_local = nh.advertise<geometry_msgs::Vector3>(
    "visual_servo/centroid/compensated/transformed_local", 1);

  // Define Subscribers
  _subOdom = nh.subscribe("odometry", 1, &uav_reference::VisualServo::odomCb, this);
  _subYawError =
    nh.subscribe("yaw_error", 1, &uav_reference::VisualServo::yawErrorCb, this);
  _subPatchCentroid = nh.subscribe(
    "centroid_point", 1, &uav_reference::VisualServo::targetCentroidCb, this);
  _subVisualServoProcessValuesMsg = nh.subscribe("VisualServoProcessValueTopic",
    1,
    &uav_reference::VisualServo::VisualServoProcessValuesCb,
    this);

  // Setup dynamic reconfigure
  _VSParamCallback = boost::bind(&VisualServo::visualServoParamsCb, this, _1, _2);
  _VSConfigServer.setCallback(_VSParamCallback);

  _serviceStartVisualServo = nh.advertiseService(
    "visual_servo", &uav_reference::VisualServo::startVisualServoServiceCb, this);

  _new_point.transforms = std::vector<geometry_msgs::Transform>(1);
  _new_point.velocities = std::vector<geometry_msgs::Twist>(1);
  _new_point.accelerations = std::vector<geometry_msgs::Twist>(1);
}

void uav_reference::VisualServo::initializeParameters(ros::NodeHandle &nh)
{
  ROS_WARN("CascadePID::initializeParameters()");

  bool x_armed = false;
  bool y_armed = false;
  bool z_armed = false;
  bool yaw_armed = false;
  bool initialized =
    nh.getParam("visual_servo/compensate_roll_and_pitch", _compensate_roll_and_pitch)
    && nh.getParam("visual_servo/yaw_added_offset", _yawAddedOffset)
    && nh.getParam("visual_servo/rate_limit", _rateLimit) &&

    nh.getParam("visual_servo/pid_x/x_armed", x_armed)
    && nh.getParam("visual_servo/pid_y/y_armed", y_armed)
    && nh.getParam("visual_servo/pid_z/z_armed", z_armed)
    && nh.getParam("visual_servo/pid_yaw/yaw_armed", yaw_armed)
    && nh.getParam("visual_servo/pid_x/deadzone_x", _deadzone_x)
    && nh.getParam("visual_servo/pid_y/deadzone_y", _deadzone_y)
    && nh.getParam("visual_servo/pid_z/deadzone_z", _deadzone_z)
    && nh.getParam("visual_servo/pid_yaw/deadzone_yaw", _deadzone_yaw) &&

    nh.getParam("visual_servo/camera/position/x", _cameraPose.position.x)
    && nh.getParam("visual_servo/camera/position/y", _cameraPose.position.y)
    && nh.getParam("visual_servo/camera/position/z", _cameraPose.position.z) &&

    nh.getParam("visual_servo/camera/orientation/x", _cameraPose.orientation.x)
    && nh.getParam("visual_servo/camera/orientation/y", _cameraPose.orientation.y)
    && nh.getParam("visual_servo/camera/orientation/z", _cameraPose.orientation.z)
    && nh.getParam("visual_servo/camera/orientation/w", _cameraPose.orientation.w);

  ROS_INFO_COND(_compensate_roll_and_pitch, "VS - Roll and pitch compensation is active");
  ROS_INFO("VS - yaw offset %.3f", _yawAddedOffset);
  ROS_INFO("VS - deadzones x,y,z,yaw = [%.3f, %.3f, %.3f, %.3f]",
    _deadzone_x,
    _deadzone_y,
    _deadzone_z,
    _deadzone_yaw);
  ROS_INFO("VS - Camera Pose [%.3f, %.3f, %.3f] - [%.3f, %.3f, %.3f, %.3f]",
    _cameraPose.position.x,
    _cameraPose.position.y,
    _cameraPose.position.z,
    _cameraPose.orientation.x,
    _cameraPose.orientation.y,
    _cameraPose.orientation.z,
    _cameraPose.orientation.w);

  if (x_armed) { _x_axis_PID.initializeParameters(nh, "visual_servo/pid_x"); }
  if (y_armed) { _y_axis_PID.initializeParameters(nh, "visual_servo/pid_y"); }
  if (z_armed) { _z_axis_PID.initializeParameters(nh, "visual_servo/pid_z"); }
  if (yaw_armed) { _yaw_PID.initializeParameters(nh, "visual_servo/pid_yaw"); }

  if (!initialized) {
    ROS_FATAL("VisualServo::initalizeParameters() - failed to initialize parameters");
    throw std::runtime_error("VisualServo parameters not properly initialized.");
  }

  brick_pickup_sm::VisualServoParametersConfig cfg;
  _VSConfigServer.getConfigDefault(cfg);

  cfg.x_armed = x_armed;
  if (x_armed) {
    cfg.k_p_x = _x_axis_PID.get_kp();
    cfg.k_i_x = _x_axis_PID.get_ki();
    cfg.k_d_x = _x_axis_PID.get_kd();
    cfg.saturation_x = _x_axis_PID.get_lim_high();
    cfg.deadzone_x = _deadzone_x;
  }

  cfg.y_armed = y_armed;
  if (y_armed) {
    cfg.k_p_y = _y_axis_PID.get_kp();
    cfg.k_i_y = _y_axis_PID.get_ki();
    cfg.k_d_y = _y_axis_PID.get_kd();
    cfg.saturation_y = _y_axis_PID.get_lim_high();
    cfg.deadzone_y = _deadzone_y;
  }

  cfg.z_armed = z_armed;
  if (z_armed) {
    cfg.k_p_z = _z_axis_PID.get_kp();
    cfg.k_i_z = _z_axis_PID.get_ki();
    cfg.k_d_z = _z_axis_PID.get_kd();
    cfg.saturation_z = _z_axis_PID.get_lim_high();
    cfg.deadzone_z = _deadzone_z;
  }

  cfg.yaw_armed = yaw_armed;
  if (yaw_armed) {
    cfg.k_p_yaw = _yaw_PID.get_kp();
    cfg.k_i_yaw = _yaw_PID.get_ki();
    cfg.k_d_yaw = _yaw_PID.get_kd();
    cfg.saturation_yaw = _yaw_PID.get_lim_high();
    cfg.deadzone_yaw = _deadzone_yaw;
  }

  cfg.rate_limit = _rateLimit;
  cfg.compensate_roll_and_pitch = _compensate_roll_and_pitch;
  cfg.yaw_added_offset = _yawAddedOffset;
  cfg.camera_x = _cameraPose.position.x;
  cfg.camera_y = _cameraPose.position.y;
  cfg.camera_z = _cameraPose.position.z;
  cfg.camera_qx = _cameraPose.orientation.x;
  cfg.camera_qy = _cameraPose.orientation.y;
  cfg.camera_qz = _cameraPose.orientation.z;
  cfg.camera_qw = _cameraPose.orientation.w;

  _VSConfigServer.updateConfig(cfg);
}


bool uav_reference::VisualServo::startVisualServoServiceCb(
  std_srvs::SetBool::Request &request,
  std_srvs::SetBool::Response &response)
{
  if (request.data) {
    if (!isVisualServoEnabled()) { ROS_INFO("UAV VisualServo - enabling visual servo."); }
    _visualServoEnabled = true;
    _yaw_PID.resetIntegrator();
    _x_axis_PID.resetIntegrator();
    _y_axis_PID.resetIntegrator();
    response.message = "Visual servo enabled.";
  } else {
    if (isVisualServoEnabled()) { ROS_INFO("UAV VisualServo - disabling visual servo."); }
    _visualServoEnabled = false;
    _yaw_PID.resetIntegrator();
    _x_axis_PID.resetIntegrator();
    _y_axis_PID.resetIntegrator();
    response.message = "Visual servo disabled.";
  }

  _x_frozen = false;
  _y_frozen = false;
  _yaw_frozen = false;
  response.success = _visualServoEnabled;
  if (_visualServoEnabled) {
    _setpointPosition[0] = _uavPos[0];
    _setpointPosition[1] = _uavPos[1];
    _setpointPosition[2] = _uavPos[2];
  }
  return true;
}

void VisualServo::visualServoParamsCb(
  brick_pickup_sm::VisualServoParametersConfig &configMsg,
  uint32_t /* unused */)
{
  ROS_WARN("VisualServo::parametersCallback");
  _deadzone_x = configMsg.deadzone_x;
  _deadzone_y = configMsg.deadzone_y;
  _deadzone_yaw = configMsg.deadzone_yaw;
  _deadzone_z = configMsg.deadzone_z;

  _x_axis_PID.set_kp(configMsg.k_p_x);
  _x_axis_PID.set_ki(configMsg.k_i_x);
  _x_axis_PID.set_kd(configMsg.k_d_x);
  _x_axis_PID.set_lim_high(configMsg.saturation_x);
  _x_axis_PID.set_lim_low(-configMsg.saturation_x);

  if (!configMsg.x_armed) {
    _x_axis_PID.set_kp(0);
    _x_axis_PID.set_ki(0);
    _x_axis_PID.set_kd(0);
    _x_axis_PID.resetIntegrator();
  }

  _y_axis_PID.set_kp(configMsg.k_p_y);
  _y_axis_PID.set_ki(configMsg.k_i_y);
  _y_axis_PID.set_kd(configMsg.k_d_y);
  _y_axis_PID.set_lim_high(configMsg.saturation_y);
  _y_axis_PID.set_lim_low(-configMsg.saturation_y);

  if (!configMsg.y_armed) {
    _y_axis_PID.set_kp(0);
    _y_axis_PID.set_ki(0);
    _y_axis_PID.set_kd(0);
    _y_axis_PID.resetIntegrator();
  }

  _yaw_PID.set_kp(configMsg.k_p_yaw);
  _yaw_PID.set_ki(configMsg.k_i_yaw);
  _yaw_PID.set_kd(configMsg.k_d_yaw);
  _yaw_PID.set_lim_high(configMsg.saturation_yaw);
  _yaw_PID.set_lim_low(-configMsg.saturation_yaw);

  if (!configMsg.yaw_armed) {
    _yaw_PID.set_kp(0);
    _yaw_PID.set_ki(0);
    _yaw_PID.set_kd(0);
    _yaw_PID.resetIntegrator();
  }

  _compensate_roll_and_pitch = configMsg.compensate_roll_and_pitch;
  _yawAddedOffset = configMsg.yaw_added_offset;
  _cameraPose.position.x = configMsg.camera_x;
  _cameraPose.position.y = configMsg.camera_y;
  _cameraPose.position.z = configMsg.camera_z;
  _cameraPose.orientation.x = configMsg.camera_qx;
  _cameraPose.orientation.y = configMsg.camera_qy;
  _cameraPose.orientation.z = configMsg.camera_qz;
  _cameraPose.orientation.w = configMsg.camera_qw;
  _rateLimit = configMsg.rate_limit;
}

void VisualServo::odomCb(const nav_msgs::OdometryConstPtr &odom)
{
  _uavOdom = *odom;
  _qx = odom->pose.pose.orientation.x;
  _qy = odom->pose.pose.orientation.y;
  _qz = odom->pose.pose.orientation.z;
  _qw = odom->pose.pose.orientation.w;

  _uavRoll = atan2(2 * (_qw * _qx + _qy * _qz), 1 - 2 * (_qx * _qx + _qy * _qy));
  _uavPitch = asin(2 * (_qw * _qy - _qx * _qz));

  _floatMsg.data = _uavRoll;
  _pubUavRollDebug.publish(_floatMsg);
  _floatMsg.data = _uavPitch;
  _pubUavPitchDebug.publish(_floatMsg);
}

void VisualServo::yawErrorCb(const std_msgs::Float32 &data)
{
  _error_yaw = util::wrapMinMax(-data.data - _yawAddedOffset, -M_PI_2, M_PI_2);
  std_msgs::Float32 m;
  m.data = _error_yaw;
  _pubYawErrorDebug.publish(m);
}

void VisualServo::VisualServoProcessValuesCb(
  const uav_ros_control_msgs::VisualServoProcessValues &msg)
{
  if (msg.x == 0.0) { _x_frozen = true; }
  if (!_x_frozen) {
    _uavPos[0] = msg.x;
  } else {
    _uavPos[0] = _setpointPosition[0];
  }

  if (msg.y == 0.0) { _y_frozen = true; }
  if (!_y_frozen) {
    _uavPos[1] = msg.y;
  } else {
    _uavPos[1] = _setpointPosition[1];
  }

  if (msg.yaw == 0.0) { _yaw_frozen = true; }
  if (!_yaw_frozen) {
    _uavYaw = msg.yaw;
  } else {
    _uavYaw = _setpointYaw;
  }

  _floatMsg.data = _uavYaw;
  _pubUavYawDebug.publish(_floatMsg);

  _uavPos[2] = msg.z;
}

void VisualServo::xOffsetCb(const std_msgs::Float32 &msg) { _offset_x = msg.data; }

void VisualServo::yOffsetCb(const std_msgs::Float32 &msg) { _offset_y = msg.data; }

void VisualServo::zOffsetCb(const std_msgs::Float32 &msg) { _offset_z = msg.data; }

void VisualServo::targetCentroidCb(const geometry_msgs::PointStamped &msg)
{
  _targetCentroid = msg;
  tf::Vector3 transformedTarget(
    _targetCentroid.point.x, _targetCentroid.point.y, _targetCentroid.point.z);

  geometry_msgs::Vector3 localCompMsg;
  localCompMsg.x = -1;
  localCompMsg.y = -1;
  localCompMsg.z = -1;

  geometry_msgs::Vector3 localCentroidMsg;
  localCentroidMsg.x = -1;
  localCentroidMsg.y = -1;
  localCentroidMsg.z = -1;

  // Todo transform the point from the camera reference frame into the UAV reference
  // frame.
  if (_compensate_roll_and_pitch && _targetCentroid.point.x != -1
      && _targetCentroid.point.y != -1 && _targetCentroid.point.z != -1) {
    tf::Transform uav_to_camera;
    uav_to_camera.setOrigin(tf::Vector3(
      _cameraPose.position.x, _cameraPose.position.y, _cameraPose.position.z));
    uav_to_camera.setRotation(tf::Quaternion(_cameraPose.orientation.x,
      _cameraPose.orientation.y,
      _cameraPose.orientation.z,
      _cameraPose.orientation.w));
    transformedTarget = uav_to_camera.inverse() * transformedTarget;

    // Publish also the centroid vector wrt. the UAV base frame
    localCentroidMsg.x = transformedTarget.getX();
    localCentroidMsg.y = transformedTarget.getY();
    localCentroidMsg.z = transformedTarget.getZ();

    tf::Vector3 compVector;
    tf::Transform comp_only_attitude;
    comp_only_attitude.setRotation(tf::Quaternion(_uavOdom.pose.pose.orientation.x,
      _uavOdom.pose.pose.orientation.y,
      _uavOdom.pose.pose.orientation.z,
      _uavOdom.pose.pose.orientation.w));
    compVector = comp_only_attitude * transformedTarget;
    localCompMsg.x = compVector.getX();
    localCompMsg.y = compVector.getY();
    localCompMsg.z = compVector.getZ();

    tf::Transform compensate_attitude;
    compensate_attitude.setRotation(tf::Quaternion(_uavOdom.pose.pose.orientation.x,
      _uavOdom.pose.pose.orientation.y,
      _uavOdom.pose.pose.orientation.z,
      _uavOdom.pose.pose.orientation.w));

    compensate_attitude.setOrigin(tf::Vector3(_uavOdom.pose.pose.position.x,
      _uavOdom.pose.pose.position.y,
      _uavOdom.pose.pose.position.z));
    transformedTarget = compensate_attitude * transformedTarget;
  }

  geometry_msgs::Vector3 globalCentroidMsg;
  globalCentroidMsg.x = transformedTarget.getX();
  globalCentroidMsg.y = transformedTarget.getY();
  globalCentroidMsg.z = transformedTarget.getZ();

  _pubTransformedTarget.publish(globalCentroidMsg);
  _pubTransformedTarget_local.publish(localCentroidMsg);
  _pubTransformedTargetComp_local.publish(localCompMsg);

  _targetCentroid.point.x = transformedTarget.getX();
  _targetCentroid.point.y = transformedTarget.getY();
  _targetCentroid.point.z = transformedTarget.getZ();
}

static double signum(double val)
{
  if (val >= 0) { return 1; }
  return -1;
}

void VisualServo::updateSetpoint()
{
  double move_forward = 0.0;
  double move_left = 0.0;
  double change_yaw = 0.0;

  if (_targetCentroid.point.x != -1 && _targetCentroid.point.y != -1
      && _targetCentroid.point.z != -1) {

    // x and y are in the UAV reference frame
    if (!_x_frozen) {
      move_forward = _x_axis_PID.compute(
        _targetCentroid.point.x, _uavOdom.pose.pose.position.x, 1 / _rate);
    }
    if (!_y_frozen) {
      move_left = _y_axis_PID.compute(
        _targetCentroid.point.y, _uavOdom.pose.pose.position.y, 1 / _rate);
    }
    if (!_yaw_frozen) { change_yaw = _yaw_PID.compute(0, _error_yaw, 1 / _rate); }

    // Do some basic rate limiter
    const double newSetpoint_0 = _uavPos[0] + move_forward;
    const double newSetpoint_1 = _uavPos[1] + move_left;

    static constexpr double DT = 0.02;
    double rate_0 = fabs(newSetpoint_0 - _setpointPosition[0]) / DT;
    double rate_1 = fabs(newSetpoint_1 - _setpointPosition[1]) / DT;

    if (rate_0 > _rateLimit) {
      _setpointPosition[0] =
        _setpointPosition[0]
        + signum(newSetpoint_0 - _setpointPosition[0]) * _rateLimit * DT;
    } else {
      _setpointPosition[0] = newSetpoint_0;
    }

    if (rate_1 > _rateLimit) {
      _setpointPosition[1] =
        _setpointPosition[1]
        + signum(newSetpoint_1 - _setpointPosition[1]) * _rateLimit * DT;
    } else {
      _setpointPosition[1] = newSetpoint_1;
    }
  } else {
    _setpointPosition[0] = _uavPos[0];
    _setpointPosition[1] = _uavPos[1];
  }


  _setpointPosition[2] = _uavPos[2];
  _setpointYaw = _uavYaw + change_yaw;

  _floatMsg.data = change_yaw;
  _pubChangeYawDebug.publish(_floatMsg);

  _moveLeftMsg.data = move_left;
  _changeYawMsg.data = change_yaw;
  _moveForwardMsg.data = move_forward;

  _pubMoveLeft.publish(_moveLeftMsg);
  _pubChangeYaw.publish(_changeYawMsg);
  _pubMoveForward.publish(_moveForwardMsg);
}

void VisualServo::publishNewSetpoint()
{
  tf2::Quaternion q;
  q.setEulerZYX(_setpointYaw, 0.0, 0.0);

  _new_point.transforms[0].translation.x = _setpointPosition[0];
  _new_point.transforms[0].translation.y = _setpointPosition[1];
  _new_point.transforms[0].translation.z = _setpointPosition[2];
  _new_point.transforms[0].rotation.x = q.getX();
  _new_point.transforms[0].rotation.y = q.getY();
  _new_point.transforms[0].rotation.z = q.getZ();
  _new_point.transforms[0].rotation.w = q.getW();

  _pubNewSetpoint.publish(_new_point);
}

void VisualServo::publishStatus()
{
  _boolMsg.data = isVisualServoEnabled();
  _pubIsEnabledTopic.publish(_boolMsg);
}

bool VisualServo::isVisualServoEnabled() { return _visualServoEnabled; }

void runDefault(VisualServo &visualServoRefObj, ros::NodeHandle & /* unused */)
{
  constexpr auto rate = 50.;
  visualServoRefObj.setRate(rate);
  ros::Rate loopRate(rate);

  while (ros::ok()) {
    ros::spinOnce();
    visualServoRefObj.updateSetpoint();
    if (visualServoRefObj.isVisualServoEnabled()) {
      visualServoRefObj.publishNewSetpoint();
    }
    visualServoRefObj.publishStatus();
    loopRate.sleep();
  }
}

void runIdle(VisualServo &visualServoRefObj, ros::NodeHandle & /* unused */)
{
  constexpr auto rate = 50.;
  visualServoRefObj.setRate(rate);
  ros::spin();
}

}// namespace uav_reference
