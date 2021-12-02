#ifndef ROS2_GREMSY_HPP_
#define ROS2_GREMSY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include <gSDK/src/gimbal_interface.h>
#include <gSDK/src/serial_port.h>

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
  geometry_msgs::msg::Vector3Stamped goals_;


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
