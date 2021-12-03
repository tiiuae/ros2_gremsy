#ifndef ROS2_GREMSY_HPP_
#define ROS2_GREMSY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <tf2_eigen/tf2_eigen.h>

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

  void declareParameters();

  void gimbalStateTimerCallback();

  void gimbalGoalTimerCallback();


  /**
   * @brief Struct for device specific limits
   */
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
    const bool lock_yaw_to_vehicle=false, const double yaw_difference = 0.0)
  {
    Eigen::Vector3d gimbal_move (msg->vector.x, msg->vector.y, msg->vector.z);
    gimbal_move.x() = RAD_TO_DEG * std::fmin(
      std::fmax(
        msg->vector.z + (lock_yaw_to_vehicle ? 0.0 : yaw_difference),
        device_specifications_[model].min_roll),
      device_specifications_[model].max_roll);

    gimbal_move.y() = RAD_TO_DEG * std::fmin(
      std::fmax(
        msg->vector.y,
        device_specifications_[model].min_tilt),
      device_specifications_[model].max_tilt);

    gimbal_move.z() = RAD_TO_DEG * std::fmin(
      std::fmax(
        msg->vector.x,
        device_specifications_[model].min_pan),
      device_specifications_[model].max_pan);

    return gimbal_move;
  }

  // @brief Device 
  gremsy_model_t device_id_;

  // @brief Serial port object
  Serial_Port * serial_port_;

  // @brief Gimbal interface object
  Gimbal_Interface * gimbal_interface_;


  // @brief Publisher for IMU data from gremsy
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // @brief Publisher for encoder Vector3 data from gremsy
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr encoder_pub_;

  // @brief Publisher for mount orientation global yaw Quaternion data
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr mount_orientation_global_pub_;

  // @brief Publisher for mount orientation local yaw Quaternion data
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr mount_orientation_local_pub_;

  // @brief Subscriber for desired mount orientation Vector3
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr desired_mount_orientation_sub_;

  // @brief Store goals
  geometry_msgs::msg::Vector3Stamped::SharedPtr goal_;
  // @brief Store yaw difference
  double yaw_difference_ = 0;

  // @brief Timer for pooling data from gremsy
  rclcpp::TimerBase::SharedPtr pool_timer_;
  // @brief Timer for sending goals to gremsy
  rclcpp::TimerBase::SharedPtr goal_timer_;

  // @brief Serial COM port to use
  std::string com_port_;

  // @brief Serial baud rate to use
  int baud_rate_;

  // @brief Rate in which the gimbal data is polled and published
  double state_poll_rate_;
  // @brief Rate in which the gimbal are pushed to the gimbal
  double goal_push_rate_;
  // @brief Control mode of the gimbal
  int gimbal_mode_;
  // @brief Input mode of the gimbals tilt axis
  int tilt_axis_input_mode_;
  // @brief Input mode of the gimbals tilt roll
  int roll_axis_input_mode_;
  // @brief Input mode of the gimbals tilt pan
  int pan_axis_input_mode_;
  // @brief Input mode of the gimbals tilt pan
  bool tilt_axis_stabilize_;
  // @brief Input mode of the gimbals tilt pan
  bool roll_axis_stabilize_;
  // @brief Input mode of the gimbals tilt pan
  bool pan_axis_stabilize_;
  // @brief Uses the yaw relative to the gimbal mount to prevent drift issues. Only a light stabilization is applied.
  bool lock_yaw_to_vehicle_;

};


}


#endif  // ROS2_GREMSY_HPP_
