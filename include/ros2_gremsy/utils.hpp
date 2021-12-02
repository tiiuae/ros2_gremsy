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

#ifndef ROS2_UTILS__PARAM_UTILS_HPP_
#define ROS2_UTILS__PARAM_UTILS_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <gSDK/src/gimbal_interface.h>
#include <gSDK/src/serial_port.h>

namespace ros2_gremsy
{

// Declare a parameter that has no integer or floating point range constraints
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
  int to_value,
  int step)
{
  auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

  descriptor.name = name;
  descriptor.description = description;
  descriptor.type = type;
  descriptor.integer_range.resize(1);
  descriptor.integer_range[0].from_value = from_value;
  descriptor.integer_range[0].to_value = to_value;
  descriptor.integer_range[0].step = step;
  return descriptor;
}

control_gimbal_mode_t convertIntGimbalMode(int mode)
{   // Allows int access to the control_gimbal_mode_t struct
    switch(mode) {
        case 0 : return GIMBAL_OFF;
        case 1 : return LOCK_MODE;
        case 2 : return FOLLOW_MODE;
        default:
            return GIMBAL_OFF;
    }
}


control_gimbal_axis_input_mode_t convertIntToAxisInputMode(int mode)
{   // Allows int access to the control_gimbal_axis_input_mode_t struct
    switch(mode) {
        case 0 : return CTRL_ANGLE_BODY_FRAME;
        case 1 : return CTRL_ANGULAR_RATE;
        case 2 : return CTRL_ANGLE_ABSOLUTE_FRAME;
        default:
            return CTRL_ANGLE_ABSOLUTE_FRAME;
    }
}

}  // namespace ros2_gremsy

#endif  // ROS2_UTILS__PARAM_UTILS_HPP_
