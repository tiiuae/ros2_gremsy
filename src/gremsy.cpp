#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ros2_gremsy/gremsy.hpp"
#include "ros2_gremsy/utils.hpp"

namespace ros2_gremsy
{
using namespace std::chrono_literals;
using std::placeholders::_1;
GremsyDriver::GremsyDriver(const rclcpp::NodeOptions & options)
: Node("ros2_gremsy", options), com_port_("COM3")
{
  GremsyDriver(options, "COM3");
}

GremsyDriver::GremsyDriver(const rclcpp::NodeOptions & options, const std::string & com_port)
: Node("ros2_gremsy", options)
{

  declareParameters();
  com_port_ = this->get_parameter("com_port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  state_poll_rate_ = this->get_parameter("state_poll_rate").as_int();
  goal_push_rate_ = this->get_parameter("goal_push_rate").as_int();
  gimbal_mode_ = this->get_parameter("gimbal_mode").as_int();
  tilt_axis_input_mode_ = this->get_parameter("tilt_axis_input_mode").as_int();
  tilt_axis_stabilize_ = this->get_parameter("tilt_axis_stabilize").as_bool();
  roll_axis_input_mode_ = this->get_parameter("roll_axis_input_mode").as_int();
  roll_axis_stabilize_ = this->get_parameter("roll_axis_stabilize").as_bool();
  pan_axis_input_mode_ = this->get_parameter("pan_axis_input_mode").as_int();
  pan_axis_stabilize_ = this->get_parameter("pan_axis_stabilize").as_bool();
  lock_yaw_to_vehicle_ = this->get_parameter("lock_yaw_to_vehicle").as_bool();


  // Initialize publishers
  this->imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("~/imu", 10);
  this->encoder_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("~/encoder", 10);
  this->mount_orientation_global_pub_ =
    this->create_publisher<geometry_msgs::msg::QuaternionStamped>(
    "~/mount_orientation_global",
    10);
  this->mount_orientation_local_pub_ =
    this->create_publisher<geometry_msgs::msg::QuaternionStamped>(
    "~/mount_orientation_local",
    10);

  // Initialize subscribers
  this->desired_mount_orientation_sub_ =
    this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
    "~/mount_orientation_local", 10,
    std::bind(&GremsyDriver::desiredOrientationCallback, this, std::placeholders::_1));


  // Define SDK objects
  serial_port_ = new Serial_Port(com_port_.c_str(), baud_rate_);
  gimbal_interface_ = new Gimbal_Interface(serial_port_);

  // Start ther serial interface and the gimbal SDK
  serial_port_->start();
  gimbal_interface_->start();

  if (gimbal_interface_->get_gimbal_status().mode == GIMBAL_STATE_OFF) {
    RCLCPP_INFO(this->get_logger(), "Gimbal is off, turning it on");
    gimbal_interface_->set_gimbal_motor_mode(TURN_ON);
  }
  while (gimbal_interface_->get_gimbal_status().mode < GIMBAL_STATE_ON) {
    RCLCPP_INFO(this->get_logger(), "Waiting for gimbal to turn on");
    std::this_thread::sleep_for(100ms);
  }

  // Set gimbal control modes

  gimbal_interface_->set_gimbal_mode(convertIntGimbalMode(gimbal_mode_));

  // Set modes for each axis

  control_gimbal_axis_mode_t tilt_axis_mode, roll_axis_mode, pan_axis_mode;
  tilt_axis_mode.input_mode = convertIntToAxisInputMode(tilt_axis_input_mode_);
  tilt_axis_mode.stabilize = tilt_axis_stabilize_;
  roll_axis_mode.input_mode = convertIntToAxisInputMode(roll_axis_input_mode_);
  roll_axis_mode.stabilize = roll_axis_stabilize_;
  pan_axis_mode.input_mode = convertIntToAxisInputMode(pan_axis_input_mode_);
  pan_axis_mode.stabilize = pan_axis_stabilize_;

  gimbal_interface_->set_gimbal_axes_mode(tilt_axis_mode, roll_axis_mode, pan_axis_mode);

  pool_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / state_poll_rate_),
    std::bind(&GremsyDriver::gimbalStateTimerCallback, this));

  goal_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / goal_push_rate_),
    std::bind(&GremsyDriver::gimbalGoalTimerCallback, this));


}
GremsyDriver::~GremsyDriver()
{
  // TODO: Close serial port
}

void GremsyDriver::gimbalStateTimerCallback()
{
  RCLCPP_INFO(this->get_logger(), "Gimbal state timer callback");
}

void GremsyDriver::gimbalGoalTimerCallback()
{
  RCLCPP_INFO(this->get_logger(), "Gimbal goal timer callback");
}

void GremsyDriver::declareParameters()
{
  this->declare_parameter(
    "com_port", "/dev/ttyUSB0",
    getParamDescriptor(
      "com_port", "Serial device for the gimbal connection",
      rcl_interfaces::msg::ParameterType::PARAMETER_STRING));

  this->declare_parameter(
    "baudrate", 115200,
    getParamDescriptor(
      "baudrate", "Baudrate for the gimbal connection",
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER));

  this->declare_parameter(
    "state_poll_rate", 10.0,
    getParamDescriptor(
      "state_poll_rate", "Rate in which the gimbal data is polled and published",
      rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE, 0.0, 300.0, 1.0));

  this->declare_parameter(
    "goal_push_rate", 60.0,
    getParamDescriptor(
      "goal_push_rate", "Rate in which the gimbal are pushed to the gimbal",
      rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE, 0.0, 300.0, 1.0));

  this->declare_parameter(
    "gimbal_mode", 1,
    getParamDescriptor(
      "gimbal_mode", "Control mode of the gimbal",
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, 0, 2, 1));

  this->declare_parameter(
    "tilt_axis_input_mode", 2,
    getParamDescriptor(
      "tilt_axis_input_mode", "Input mode of the gimbals tilt axis",
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, 0, 2, 1));

  this->declare_parameter(
    "tilt_axis_stabilize", true,
    getParamDescriptor(
      "tilt_axis_stabilize", "Input mode of the gimbals tilt axis",
      rcl_interfaces::msg::ParameterType::PARAMETER_BOOL));

  this->declare_parameter(
    "roll_axis_input_mode", 2,
    getParamDescriptor(
      "roll_axis_input_mode", "Input mode of the gimbals tilt roll",
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, 0, 2, 1));

  this->declare_parameter(
    "roll_axis_stabilize", true,
    getParamDescriptor(
      "roll_axis_stabilize", "Input mode of the gimbals tilt roll",
      rcl_interfaces::msg::ParameterType::PARAMETER_BOOL));

  this->declare_parameter(
    "pan_axis_input_mode", 2,
    getParamDescriptor(
      "pan_axis_input_mode", "Input mode of the gimbals tilt pan",
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, 0, 2, 1));

  this->declare_parameter(
    "pan_axis_stabilize", true,
    getParamDescriptor(
      "pan_axis_stabilize", "Input mode of the gimbals tilt pan",
      rcl_interfaces::msg::ParameterType::PARAMETER_BOOL));

  this->declare_parameter(
    "lock_yaw_to_vehicle", true,
    getParamDescriptor(
      "lock_yaw_to_vehicle",
      "Uses the yaw relative to the gimbal mount to prevent drift issues. Only a light stabilization is applied.",
      rcl_interfaces::msg::ParameterType::PARAMETER_BOOL));
}


} // namespace ros2_gremsy
