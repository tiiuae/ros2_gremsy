// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_GREMSY__UTILS_HPP_
#define ROS2_GREMSY__UTILS_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include <../../gSDK/src/gimbal_interface.h>
#include <../../gSDK/src/serial_port.h>

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

namespace ros2_gremsy
{

// Declare a parameter that has no integer or floating point range constraints
inline rcl_interfaces::msg::ParameterDescriptor getParamDescriptor(
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

inline rcl_interfaces::msg::ParameterDescriptor getParamDescriptor(
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

inline rcl_interfaces::msg::ParameterDescriptor getParamDescriptor(
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

inline control_gimbal_mode_t convertIntGimbalMode(int mode)
{   // Allows int access to the control_gimbal_mode_t struct
  switch (mode) {
    case 0: return GIMBAL_OFF;
    case 1: return LOCK_MODE;
    case 2: return FOLLOW_MODE;
    default:
      return GIMBAL_OFF;
  }
}


inline control_gimbal_axis_input_mode_t convertIntToAxisInputMode(int mode)
{   // Allows int access to the control_gimbal_axis_input_mode_t struct
  switch (mode) {
    case 0: return CTRL_ANGLE_BODY_FRAME;
    case 1: return CTRL_ANGULAR_RATE;
    case 2: return CTRL_ANGLE_ABSOLUTE_FRAME;
    default:
      return CTRL_ANGLE_ABSOLUTE_FRAME;
  }
}
inline Eigen::Quaterniond convertXYZtoQuaternion(double roll, double pitch, double yaw)
{
  // The yaw angle is negated to match with incoming goals
  Eigen::Quaterniond quat_abs(
    Eigen::AngleAxisd(DEG_TO_RAD * roll, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(DEG_TO_RAD * pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(-DEG_TO_RAD * yaw, Eigen::Vector3d::UnitZ()));
  return quat_abs;
}

inline Eigen::Vector3d convertQuaterniontoZYX(double x, double y, double z, double w)
{
  Eigen::Vector3d result;

  double sinr_cosp = 2.0 * (w * x + y * z);
  double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  result[0] = atan2(sinr_cosp, cosr_cosp);

  double sinp = 2.0 * (w * y - z * x);
  if (abs(sinp) >= 1) {
    result[1] = copysign(M_PI / 2, sinp);
  } else {
    result[1] = asin(sinp);
  }

  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  result[2] = atan2(siny_cosp, cosy_cosp);

  return result;
}

inline sensor_msgs::msg::Imu convertImuMavlinkMessageToROSMessage(mavlink_raw_imu_t message)
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

inline geometry_msgs::msg::QuaternionStamped stampQuaternion(
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

inline double limitAngle(double angle, double min, double max)
{
  if (angle > max) {
    return max;
  } else if (angle < min) {
    return min;
  }
  return angle;
}

inline double limitAngle(double angle, double range)
{
  return limitAngle(angle, -range, range);
}


}  // namespace ros2_gremsy

#endif  // ROS2_GREMSY__UTILS_HPP_
