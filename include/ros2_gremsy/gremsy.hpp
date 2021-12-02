#ifndef ROS2_GREMSY_HPP_
#define ROS2_GREMSY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <tf2_eigen/tf2_eigen.h>

//#include "ros2_gremsy/utils.hpp"
#include <../../gSDK/src/gimbal_interface.h>
#include <../../gSDK/src/serial_port.h>

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)
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
  geometry_msgs::msg::Vector3Stamped::SharedPtr goals_;
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
  rcl_interfaces::msg::ParameterDescriptor getParamDescriptor(
    const std::string & name,
    const std::string & description,
    const uint8_t & type)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.type = type;
    return descriptor;
  }

  rcl_interfaces::msg::ParameterDescriptor getParamDescriptor(
    const std::string & name,
    const std::string & description,
    const uint8_t & type,
    double from_value,
    double to_value,
    double step)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.type = type;
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = from_value;
    descriptor.floating_point_range[0].to_value = to_value;
    descriptor.floating_point_range[0].step = step;
    return descriptor;
  }

  rcl_interfaces::msg::ParameterDescriptor getParamDescriptor(
    const std::string & name,
    const std::string & description,
    const uint8_t & type,
    int from_value,
    int to_value)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.type = type;
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = from_value;
    descriptor.integer_range[0].to_value = to_value;
    descriptor.integer_range[0].step = 1;
    return descriptor;
  }

  control_gimbal_mode_t convertIntGimbalMode(int mode)
  { // Allows int access to the control_gimbal_mode_t struct
    switch (mode) {
      case 0: return GIMBAL_OFF;
      case 1: return LOCK_MODE;
      case 2: return FOLLOW_MODE;
      default:
        return GIMBAL_OFF;
    }
  }


  control_gimbal_axis_input_mode_t convertIntToAxisInputMode(int mode)
  { // Allows int access to the control_gimbal_axis_input_mode_t struct
    switch (mode) {
      case 0: return CTRL_ANGLE_BODY_FRAME;
      case 1: return CTRL_ANGULAR_RATE;
      case 2: return CTRL_ANGLE_ABSOLUTE_FRAME;
      default:
        return CTRL_ANGLE_ABSOLUTE_FRAME;
    }
  }
  Eigen::Quaterniond convertYXZtoQuaternion(double roll, double pitch, double yaw)
  {
    Eigen::Quaterniond quat_abs(
      Eigen::AngleAxisd(-DEG_TO_RAD * pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(-DEG_TO_RAD * roll, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(DEG_TO_RAD * yaw, Eigen::Vector3d::UnitZ()));
    return quat_abs;
  }

  sensor_msgs::msg::Imu convertImuMavlinkMessageToROSMessage(mavlink_raw_imu_t message)
  {
    sensor_msgs::msg::Imu imu_message;

    // Set accelaration data
    imu_message.linear_acceleration.x = message.xacc;
    imu_message.linear_acceleration.y = message.yacc;
    imu_message.linear_acceleration.z = message.zacc;

    // Set gyro data
    imu_message.angular_velocity.x = message.xgyro;
    imu_message.angular_velocity.y = message.ygyro;
    imu_message.angular_velocity.z = message.zgyro;

    return imu_message;
  }

  geometry_msgs::msg::QuaternionStamped stampQuaternion(
    geometry_msgs::msg::Quaternion quat,
    std::string frame_id,
    builtin_interfaces::msg::Time time)
  {
    geometry_msgs::msg::QuaternionStamped quat_stamped;
    quat_stamped.header.frame_id = frame_id;
    quat_stamped.header.stamp = time;
    quat_stamped.quaternion = quat;
    return quat_stamped;
  }

};


}


#endif  // ROS2_GREMSY_HPP_
