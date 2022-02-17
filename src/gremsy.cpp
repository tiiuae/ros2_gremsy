#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ros2_gremsy/gremsy.hpp"

namespace ros2_gremsy
{
using namespace std::chrono_literals;
using std::placeholders::_1;
GremsyDriver::GremsyDriver(const rclcpp::NodeOptions & options)
: Node("ros2_gremsy", options), com_port_("/dev/ttyUSB0"), use_ros_time_(true)
{
  GremsyDriver(options, "/dev/ttyUSB0");
  
}

GremsyDriver::GremsyDriver(const rclcpp::NodeOptions & options, const std::string & com_port)
: Node("ros2_gremsy", options), use_ros_time_(true)
{

  declareParameters();
  device_id_ = gremsy_model_t(this->get_parameter("device_id").as_int());
  com_port_ = this->get_parameter("com_port").as_string();
  baud_rate_ = this->get_parameter("baudrate").as_int();
  state_poll_rate_ = this->get_parameter("state_poll_rate").as_double();
  goal_push_rate_ = this->get_parameter("goal_push_rate").as_double();
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
    this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "~/gimbal_goal", 10,
    std::bind(&GremsyDriver::desiredOrientationCallback, this, std::placeholders::_1));
    
  this->target_pose_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/gimbal_goal_pose", 10,
    std::bind(&GremsyDriver::targetPoseCallback, this, std::placeholders::_1));

  mount_orientation_ = Eigen::Vector3d(0, 0, 0);
  mount_orientation_absolute_ = Eigen::Vector3d(0, 0, 0);

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
  RCLCPP_DEBUG(this->get_logger(), "Gimbal state timer callback");
  // Publish Gimbal IMU
  mavlink_raw_imu_t imu_mav = gimbal_interface_->get_gimbal_raw_imu();
  imu_mav.time_usec = gimbal_interface_->get_gimbal_time_stamps().raw_imu;
  sensor_msgs::msg::Imu imu_ros_mag = convertImuMavlinkMessageToROSMessage(imu_mav);

  imu_ros_mag.header.stamp = use_ros_time_ ? this->get_clock()->now() : rclcpp::Time(
    (int64_t)imu_mav.time_usec * 1000UL);
  imu_pub_->publish(imu_ros_mag);

  // Publish Gimbal Encoder Values
  mavlink_mount_status_t mount_status = gimbal_interface_->get_gimbal_mount_status();
  uint64_t mnt_status_time_stamp = gimbal_interface_->get_gimbal_time_stamps().mount_status;
  // TODO: Confirm that the mount status timestamp is in microseconds

  geometry_msgs::msg::Vector3Stamped encoder_ros_msg;

  encoder_ros_msg.header.stamp = use_ros_time_ ? this->get_clock()->now() : rclcpp::Time(
    (int64_t)mnt_status_time_stamp * 1000UL);

  encoder_ros_msg.vector.x = ((float) mount_status.pointing_b) * DEG_TO_RAD;
  encoder_ros_msg.vector.y = ((float) mount_status.pointing_a) * DEG_TO_RAD;
  encoder_ros_msg.vector.z = ((float) mount_status.pointing_c) * DEG_TO_RAD;
  // encoder_ros_msg.header TODO time stamps

  encoder_pub_->publish(encoder_ros_msg);

  // Get Mount Orientation
  mavlink_mount_orientation_t mount_orientation = gimbal_interface_->get_gimbal_mount_orientation();
  mount_orientation.time_boot_ms = gimbal_interface_->get_gimbal_time_stamps().mount_orientation;

  rclcpp::Time stamp = use_ros_time_ ? this->get_clock()->now() : rclcpp::Time(
    (int64_t)mount_orientation.time_boot_ms * 1000000UL);

  yaw_difference_ = DEG_TO_RAD * (mount_orientation.yaw_absolute - mount_orientation.yaw);

  // Publish Camera Mount Orientation in global frame (drifting)
  mount_orientation_global_pub_->publish(
    stampQuaternion(
      tf2::toMsg(
        convertYXZtoQuaternion(
          mount_orientation.roll,
          mount_orientation.pitch,
          mount_orientation.yaw_absolute)),
      "gimbal_link", stamp));
  mount_orientation_absolute_.x() = mount_orientation.roll * DEG_TO_RAD;
  mount_orientation_absolute_.y() = mount_orientation.pitch * DEG_TO_RAD;
  mount_orientation_absolute_.z() = mount_orientation.yaw_absolute * DEG_TO_RAD;

  // Publish Camera Mount Orientation in local frame (yaw relative to vehicle)
  mount_orientation_local_pub_->publish(
    stampQuaternion(
      tf2::toMsg(
        convertYXZtoQuaternion(
          mount_orientation.roll,
          mount_orientation.pitch,
          mount_orientation.yaw)),
      "gimbal_link", stamp));
  mount_orientation_.x() = mount_orientation.roll * DEG_TO_RAD;
  mount_orientation_.y() = mount_orientation.pitch * DEG_TO_RAD;
  mount_orientation_.z() = mount_orientation.yaw * DEG_TO_RAD;
}

void GremsyDriver::gimbalGoalTimerCallback()
{
  RCLCPP_DEBUG(this->get_logger(), "Gimbal goal timer callback");
  if (goal_) {
    RCLCPP_DEBUG(this->get_logger(), "Gimbal desired orientation is: %f, %f, %f",
      goal_->vector.x, goal_->vector.y, goal_->vector.z);

    Eigen::Vector3d desired_orientation_eigen = prepareGimbalMove(
      goal_, device_id_, lock_yaw_to_vehicle_, yaw_difference_);
    RCLCPP_DEBUG(this->get_logger(), "Desired orientation: %f, %f, %f",
      desired_orientation_eigen(0), desired_orientation_eigen(1), desired_orientation_eigen(2));
    gimbal_interface_->set_gimbal_move(
      desired_orientation_eigen.y(),
      desired_orientation_eigen.x(),
      desired_orientation_eigen.z());
    goal_ = nullptr;
  }
}

void GremsyDriver::desiredOrientationCallback(
  const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  goal_ = msg;
}

void GremsyDriver::targetPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (strstr(msg->header.frame_id.c_str(), "camera") != NULL) {
    geometry_msgs::msg::Vector3Stamped relative = relativeToGimbalFromPose(msg, mount_orientation_);
    RCLCPP_INFO(this->get_logger(), "Msg: %f, %f Rel: %f, %f, %f Mount: %f, %f, %f",
      msg->pose.position.x, msg->pose.position.y, relative.vector.x, relative.vector.y, relative.vector.z, mount_orientation_.x(), mount_orientation_.y(), mount_orientation_.z());
    relative.header.frame_id = "gimbal_link";
    std::shared_ptr<geometry_msgs::msg::Vector3Stamped> relative_ptr =
      std::make_shared<geometry_msgs::msg::Vector3Stamped>(relative);
    goal_ = relative_ptr;

  }
}

void GremsyDriver::declareParameters()
{
  this->declare_parameter(
    "device_id", 0,
    getParamDescriptor(
      "device_id", "Device id- 0: MIO, 1: S1, 2: T3V3, 3: T7",
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, 0, 3));

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
    "state_poll_rate", 50.0,
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
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, 0, 2));

  this->declare_parameter(
    "tilt_axis_input_mode", 2,
    getParamDescriptor(
      "tilt_axis_input_mode",
      "Input mode of the gimbals tilt axis, 0: angle body, 1: ground angular rate, 2: ground absolute angle",
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, 0, 2));

  this->declare_parameter(
    "tilt_axis_stabilize", true,
    getParamDescriptor(
      "tilt_axis_stabilize", "Input mode of the gimbals tilt axis",
      rcl_interfaces::msg::ParameterType::PARAMETER_BOOL));

  this->declare_parameter(
    "roll_axis_input_mode", 2,
    getParamDescriptor(
      "roll_axis_input_mode",
      "Input mode of the gimbals tilt roll, 0: angle body, 1: ground angular rate, 2: ground absolute angle",
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, 0, 2));

  this->declare_parameter(
    "roll_axis_stabilize", true,
    getParamDescriptor(
      "roll_axis_stabilize", "Input mode of the gimbals tilt roll",
      rcl_interfaces::msg::ParameterType::PARAMETER_BOOL));

  this->declare_parameter(
    "pan_axis_input_mode", 2,
    getParamDescriptor(
      "pan_axis_input_mode",
      "Input mode of the gimbals tilt pan, 0: angle body, 1: ground angular rate, 2: ground absolute angle",
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, 0, 2));

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
