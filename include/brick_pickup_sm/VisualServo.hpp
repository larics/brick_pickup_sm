//
// Created by robert on 20. 09. 2019..
//

#ifndef UAV_ROS_CONTROL_VISUALSERVO_H
#define UAV_ROS_CONTROL_VISUALSERVO_H

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_srvs/SetBool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>

#include <brick_pickup_sm/VisualServoParametersConfig.h>
#include <dynamic_reconfigure/server.h>
#include <uav_ros_control/control/PID.hpp>
#include <uav_ros_control/filters/NonlinearFilters.hpp>
#include <uav_ros_control_msgs/VisualServoProcessValues.h>

namespace uav_reference {
/**
 * Name of dynamic reconfigure node.
 */
#define VISUAL_SERVO_DYN_RECONF "visual_servo_config"

/**
 * Publish UAV reference based on the error reported by the drone's color filter.
 * The reference is published in the global coordinate system.
 **/
class VisualServo
{
public:
  /**
   * Default constructor.
   */
  VisualServo(ros::NodeHandle &);

  /**
   * Update position setpoint.
   */
  void updateSetpoint();

  /**
   * Publish new setpoint as MultiDOFJointTrajectoryPoint
   */
  void publishNewSetpoint();
  bool isVisualServoEnabled();
  void publishStatus();
  void setRate(double new_rate) { _rate = new_rate; }
  void initializeParameters(ros::NodeHandle &);

private:
  /**
   * Callback function for StartVisualServo service
   */
  bool startVisualServoServiceCb(std_srvs::SetBool::Request &request,
    std_srvs::SetBool::Response &response);

  /**
   * Odometry callback function for extracting the UAV's pose.
   */
  void odomCb(const nav_msgs::OdometryConstPtr &);
  void yawErrorCb(const std_msgs::Float32 &);
  void VisualServoProcessValuesCb(const uav_ros_control_msgs::VisualServoProcessValues &);
  void xOffsetCb(const std_msgs::Float32 &);
  void yOffsetCb(const std_msgs::Float32 &);
  void zOffsetCb(const std_msgs::Float32 &);
  void targetCentroidCb(const geometry_msgs::PointStamped &);

  // X and Y axes of the image coordinate frame.
  PID _x_axis_PID{ "x-axis" }, _y_axis_PID{ "y-axis" }, _z_axis_PID{ "z-axis" },
    _yaw_PID{ "yaw" };

  int _n_contours = 0;
  std::array<double, 3> _uavPos{ 0.0, 0.0, 0.0 };
  std::array<double, 3> _setpointPosition{ 0.0, 0.0, 0.0 };
  geometry_msgs::PointStamped _targetCentroid;
  geometry_msgs::Pose _cameraPose;
  nav_msgs::Odometry _uavOdom;
  double _error_x = 0, _error_y = 0, _error_z = 0, _error_yaw = 0, _offset_x = 0;
  double _offset_y = 0, _offset_z = 0, _deadzone_x = 0, _deadzone_y = 0, _deadzone_z = 0,
         _deadzone_yaw = 0;
  double _qx = 0, _qy = 0, _qz = 0, _qw = 0, _uavYaw = 0, _uavRoll = 0, _uavPitch = 0,
         _setpointYaw = 0, _yawAddedOffset = 0;
  double _rate = 0, _rateLimit = 0;

  bool _visualServoEnabled = false, _compensate_roll_and_pitch = false;
  bool _x_frozen = false, _y_frozen = false, _yaw_frozen = false;
  bool _compensate_camera_nonlinearity = false;

  /** Publishers */
  ros::Publisher _pubNewSetpoint;
  trajectory_msgs::MultiDOFJointTrajectoryPoint _new_point;

  // Status topic
  ros::Publisher _pubIsEnabledTopic;
  std_msgs::Bool _boolMsg;

  // Topics for direct rotor control
  ros::Publisher _pubMoveLeft, _pubMoveForward, _pubChangeYaw;
  std_msgs::Float32 _moveLeftMsg, _moveForwardMsg, _changeYawMsg;

  // Topics for debugging
  ros::Publisher _pubUavYawDebug, _pubChangeYawDebug, _pubYawErrorDebug;
  ros::Publisher _pubUavRollDebug, _pubUavPitchDebug;
  ros::Publisher _pubTransformedTarget, _pubTransformedTarget_local,
    _pubTransformedTargetComp_local;
  std_msgs::Float32 _floatMsg;

  // Brick errors publisher
  ros::Publisher _pubXError, _pubYError;

  /** Subscribers */
  ros::Subscriber _subOdom, _subImu;
  ros::Subscriber _subYawError, _subNContours;
  ros::Subscriber _subVisualServoProcessValuesMsg;
  ros::Subscriber _subPatchCentroid;

  uav_ros_control_msgs::VisualServoProcessValues VisualServoProcessValuesMsg;

  /** Services */
  ros::ServiceServer _serviceStartVisualServo;
  std_srvs::SetBool::Request _setBoolRequest;
  std_srvs::SetBool::Response _setBoolResponse;

  void visualServoParamsCb(brick_pickup_sm::VisualServoParametersConfig &configMsg,
    uint32_t level);

  /** Define Dynamic Reconfigure parameters **/
  boost::recursive_mutex _VSConfigMutex;
  dynamic_reconfigure::Server<brick_pickup_sm::VisualServoParametersConfig>
    _VSConfigServer{ _VSConfigMutex, ros::NodeHandle(VISUAL_SERVO_DYN_RECONF) };
  dynamic_reconfigure::Server<brick_pickup_sm::VisualServoParametersConfig>::CallbackType
    _VSParamCallback;
};

void runDefault(VisualServo &, ros::NodeHandle &);
void runIdle(VisualServo &, ros::NodeHandle &);
}// namespace uav_reference

#endif// UAV_ROS_CONTROL_VISUALSERVO_H
