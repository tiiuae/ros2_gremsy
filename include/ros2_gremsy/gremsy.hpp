#ifndef ROS2_GREMSY_HPP_
#define ROS2_GREMSY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "ros2_gremsy/utils.hpp"
#include <../../gSDK/src/gimbal_interface.h>
#include <../../gSDK/src/serial_port.h>

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

enum gremsy_model_t
{
  GREMSY_MIO = 0,
  GREMSY_S1,
  GREMSY_T3V3,
  GREMSY_T7,
  NUM_OF_MODELS
};


namespace ros2_gremsy
{

class GremsyDriver : public rclcpp::Node
{
public:
  GremsyDriver(const rclcpp::NodeOptions & options);
  GremsyDriver(const rclcpp::NodeOptions & options, const std::string & serial_port);
  ~GremsyDriver();

private:
  /**
   * @brief Desired mount orientation callback Vector3
   * @param msg Vector3Stamped message
   */
  void desiredOrientationCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

  /**
   * @brief Desired mount orientation callback Quaternion
   * @param msg QuaternionStamped message
   */
  void desiredOrientationQuaternionCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg);

  /**
   * @brief Enable lock mode callback
   * @param request SetBool request. Only field is bool data. true
   * @param response SetBool response. Has fields bool success and string message.
   * This callback will change the gimbal mode unless the new mode is already active.
   * response->success is false if new mode is already active, true otherwise.
   */
  void enableLockModeCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              const std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /// Declare Parameters for the nodes
  void declareParameters();

  /**
   * @brief State of the gimbal will be pooled by a timer
   * This callback will get IMU, encoders, and mount orientation
   */
  void gimbalStateTimerCallback();

  /**
   * @brief This callback will get the last command from ROS2 topic,
   * and send it to the gimbal
   */
  void gimbalGoalTimerCallback();

  /**
   * @brief Set message rates for polled gimbal status
   */
  void set_msg_rates();


  /// Struct for device specific limits
  struct
  {
    const gremsy_model_t device_name;
    const double min_pan;
    const double max_pan;
    const double min_tilt;
    const double max_tilt;
    const double min_roll;
    const double max_roll;
  } device_specifications_[NUM_OF_MODELS] = {
    {GREMSY_MIO, -325.0, 325.0, -120.0, 120.0, -40.0, 40.0},
    {GREMSY_S1, -345.0, 345.0, -120.0, 120.0, -45.0, 45.0},
    {GREMSY_T3V3, -345.0, 345.0, -120.0, 120.0, -45.0, 45.0},
    {GREMSY_T7, -300.0, 300.0, -120.0, 120.0, -45.0, 45.0},
  };

  /**
   * @brief Limit the desired orientation to the device specifications
   * Enforce gimbal limits on the desired orientation
   * @param msg Vector3Stamped desired orientation message
   * @param model gremsy_model_t for specific limits of the device
   * @param lock_yaw_to_vehicle If true, the yaw will be locked to the vehicle's yaw
   * @param yaw_difference Mount yaw orientation absolute difference from mount yaw
   * @return Vector3d of desired orientation in degrees (x:roll, y:pitch, z:yaw)
   *
   */
  Eigen::Vector3d prepareGimbalMove(
    const geometry_msgs::msg::Vector3Stamped::SharedPtr & msg, const int model,
    const bool lock_yaw_to_vehicle = false, const double yaw_difference = 0.0)
  {
    Eigen::Vector3d gimbal_move(msg->vector.x, msg->vector.y, msg->vector.z);
    gimbal_move.x() = std::fmin(
      std::fmax(
        RAD_TO_DEG * msg->vector.x,
        device_specifications_[model].min_roll),
      device_specifications_[model].max_roll);

    gimbal_move.y() = std::fmin(
      std::fmax(
        RAD_TO_DEG * msg->vector.y,
        device_specifications_[model].min_tilt),
      device_specifications_[model].max_tilt);

    gimbal_move.z() = std::fmin(
      std::fmax(
        RAD_TO_DEG * (msg->vector.z + (lock_yaw_to_vehicle ? 0.0 : yaw_difference)),
        device_specifications_[model].min_pan),
      device_specifications_[model].max_pan);

    return gimbal_move;
  }

  /// Device
  gremsy_model_t device_id_;

  /// Serial port object
  Serial_Port * serial_port_;

  /// Gimbal interface object
  Gimbal_Interface * gimbal_interface_;


  /// Publisher for IMU data from gremsy
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  /// Publisher for encoder Vector3 data from gremsy
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr encoder_pub_;

  /// Publisher for mount orientation global yaw Quaternion data
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr mount_orientation_global_pub_;

  /// Publisher for mount orientation local yaw Quaternion data
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr mount_orientation_local_pub_;

  /// Subscriber for desired mount orientation Vector3
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr desired_mount_orientation_sub_;

  /// Subscriber for desired mount orientation Quaternion
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr desired_mount_orientation_quaternion_sub_;

  /// Service for gimbal mode change
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_lock_mode_service_;

  /// Store goals
  geometry_msgs::msg::Vector3Stamped::SharedPtr goal_;
  /// Store yaw difference
  double yaw_difference_ = 0;

  /// Timer for pooling data from gremsy
  rclcpp::TimerBase::SharedPtr pool_timer_;
  /// Timer for sending goals to gremsy
  rclcpp::TimerBase::SharedPtr goal_timer_;

  /// Serial COM port to use
  std::string com_port_;

  /// Serial baud rate to use
  int baud_rate_;

  /// Rate in which the gimbal data is polled and published
  double state_poll_rate_;
  /// Rate in which the gimbal are pushed to the gimbal
  double goal_push_rate_;
  /// Control mode of the gimbal
  int gimbal_mode_;
  /// Input mode of the gimbals tilt axis
  int tilt_axis_input_mode_;
  /// Input mode of the gimbals tilt roll
  int roll_axis_input_mode_;
  /// Input mode of the gimbals tilt pan
  int pan_axis_input_mode_;
  /// Input mode of the gimbals tilt pan
  bool tilt_axis_stabilize_;
  /// Input mode of the gimbals tilt pan
  bool roll_axis_stabilize_;
  /// Input mode of the gimbals tilt pan
  bool pan_axis_stabilize_;
  /// Uses the yaw relative to the gimbal mount to prevent drift issues. Only a light stabilization is applied.
  bool lock_yaw_to_vehicle_;
  /// Time source for the published messages, ros node time is true
  bool use_ros_time_;

};


}


#endif  // ROS2_GREMSY_HPP_
