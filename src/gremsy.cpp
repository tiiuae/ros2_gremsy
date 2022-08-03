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
using std::placeholders::_2;
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

  this->desired_mount_orientation_quaternion_sub_ =
    this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
    "~/gimbal_goal_quaternion", 10,
    std::bind(&GremsyDriver::desiredOrientationQuaternionCallback, this, std::placeholders::_1));

  // Create services
  this->enable_lock_mode_service_ =
    this->create_service<std_srvs::srv::SetBool>("~/lock_mode",
    std::bind(&GremsyDriver::enableLockModeCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Define SDK objects
  serial_port_ = new Serial_Port(com_port_.c_str(), baud_rate_);
  gimbal_interface_ = new Gimbal_Interface(serial_port_);

  // Start ther serial interface and the gimbal SDK
  serial_port_->start();
  gimbal_interface_->start();

  if (gimbal_interface_->get_gimbal_status().state == Gimbal_Interface::GIMBAL_STATE_OFF) {
    RCLCPP_INFO(this->get_logger(), "Gimbal is off, turning it on");
    gimbal_interface_->set_gimbal_motor(Gimbal_Interface::TURN_ON);
  }
  while (gimbal_interface_->get_gimbal_status().state < Gimbal_Interface::GIMBAL_STATE_ON) {
    RCLCPP_INFO(this->get_logger(), "Waiting for gimbal to turn on");
    std::this_thread::sleep_for(100ms);
  }

  // Set gimbal control modes
  if (gimbal_mode_ == 1) {
    gimbal_interface_->set_gimbal_lock_mode_sync();
  } else {
    gimbal_interface_->set_gimbal_follow_mode_sync();
  }

  // Send initial goal to the starting orientation
  std::shared_ptr<geometry_msgs::msg::Vector3Stamped> message = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
  message->header.stamp = rclcpp::Time(0);
  message->header.frame_id = "";
  message->vector.x = 0;
  message->vector.y = 0;
  message->vector.z = 0;
  goal_ = message;

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

void GremsyDriver::set_msg_rates(){
  // Poll status with a higher frequency to ensure proper values for each published message
  uint8_t status_rate = (uint8_t)state_poll_rate_ * 2 + 10;
  gimbal_interface_->set_msg_encoder_rate(status_rate);
  gimbal_interface_->set_msg_mnt_orient_rate(status_rate);
  gimbal_interface_->set_msg_attitude_status_rate(status_rate);
  gimbal_interface_->set_msg_raw_imu_rate(status_rate);
  // Set encoder to send values as angles, not raw
  gimbal_interface_->set_gimbal_encoder_type_send(false);
}

void GremsyDriver::gimbalStateTimerCallback()
{
  //RCLCPP_DEBUG(this->get_logger(), "Gimbal state timer callback");
  // Publish Gimbal IMU
  Gimbal_Interface::imu_t imu_mav = gimbal_interface_->get_gimbal_raw_imu();

  sensor_msgs::msg::Imu imu_ros_mag = convertImuToROSMessage(imu_mav);

  imu_ros_mag.header.stamp = use_ros_time_ ? this->get_clock()->now() : rclcpp::Time(
    (int64_t)gimbal_interface_->get_gimbal_timestamps().raw_imu * 1000UL);
  imu_pub_->publish(imu_ros_mag);

  // Publish Gimbal Encoder Values
  attitude<int16_t> mount_status = gimbal_interface_->get_gimbal_encoder();
  uint64_t mnt_status_time_stamp = gimbal_interface_->get_gimbal_timestamps().mount_status;
  // TODO: Confirm that the mount status timestamp is in microseconds

  geometry_msgs::msg::Vector3Stamped encoder_ros_msg;

  encoder_ros_msg.header.stamp = use_ros_time_ ? this->get_clock()->now() : rclcpp::Time(
    (int64_t)mnt_status_time_stamp * 1000UL);

  encoder_ros_msg.vector.x = ((float) mount_status.roll) * DEG_TO_RAD;
  encoder_ros_msg.vector.y = ((float) mount_status.pitch) * DEG_TO_RAD;
  encoder_ros_msg.vector.z = ((float) mount_status.yaw) * DEG_TO_RAD;
  // encoder_ros_msg.header TODO time stamps

  encoder_pub_->publish(encoder_ros_msg);

  // Get Gimbal Attitude
  attitude<float> mount_orientation = gimbal_interface_->get_gimbal_attitude();
  uint64_t gimbal_attitude_time_stamp = gimbal_interface_->get_gimbal_timestamps().attitude_status;

  rclcpp::Time stamp = use_ros_time_ ? this->get_clock()->now() : rclcpp::Time(
    (int64_t)gimbal_attitude_time_stamp * 1000UL);

  // TODO: Publish global orientation. New SDK doesn't offer this by default.
  /*yaw_difference_ = DEG_TO_RAD * (mount_orientation.yaw_absolute - mount_orientation.yaw);

  // Publish Camera Mount Orientation in global frame (drifting)
  mount_orientation_global_pub_->publish(
    stampQuaternion(
      tf2::toMsg(
        convertXYZtoQuaternion(
          mount_orientation.roll,
          mount_orientation.pitch,
          mount_orientation.yaw_absolute)),
      "gimbal_link", stamp));
  */
  // Publish Camera Mount Orientation in local frame (yaw relative to vehicle)
  mount_orientation_local_pub_->publish(
    stampQuaternion(
      tf2::toMsg(
        convertXYZtoQuaternion(
          mount_orientation.roll,
          mount_orientation.pitch,
          mount_orientation.yaw)),
      "gimbal_link", stamp));
}

void GremsyDriver::gimbalGoalTimerCallback()
{
  // RCLCPP_DEBUG(this->get_logger(), "Gimbal goal timer callback");
  static bool once = true;
  if (goal_) {
    RCLCPP_DEBUG(this->get_logger(), "Gimbal desired orientation is: %f, %f, %f",
      goal_->vector.x, goal_->vector.y, goal_->vector.z);
    Eigen::Vector3d desired_orientation_eigen = prepareGimbalMove(
      goal_, device_id_, lock_yaw_to_vehicle_, yaw_difference_);
    RCLCPP_DEBUG(this->get_logger(), "Desired orientation: %f, %f, %f",
      desired_orientation_eigen(0), desired_orientation_eigen(1), desired_orientation_eigen(2));
    Gimbal_Protocol::result_t result = gimbal_interface_->set_gimbal_rotation_sync(
      desired_orientation_eigen.y(),
      desired_orientation_eigen.x(),
      desired_orientation_eigen.z());
    goal_ = nullptr;
    if (once){
      // For some reason, polling rates change weirdly after the first goal has been given to gimbal.
      // As a workaround, we set the rates after the initial goal is set.
      printf("Reseting rates \n");
      set_msg_rates();
      once = false;
    }
  }
}

void GremsyDriver::desiredOrientationCallback(
  const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "New goal received: x: '%.2f', y: '%.2f', z: '%.2f'", msg->vector.x, msg->vector.y, msg->vector.z);
  goal_ = msg;
}

void GremsyDriver::desiredOrientationQuaternionCallback(
  const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
{
  // Extract roll, pitch, yaw angles.
  // The function returns rotation values that we need to convert into orientations.
  // The easiest way to fix this is to send the conjugate of the parameter quaternion.
  Eigen::Vector3d angles = convertQuaterniontoZYX(msg->quaternion.x, msg->quaternion.y, msg->quaternion.z, -msg->quaternion.w);

  // goal_ requires a Vector3Stamped, so we convert the message type
  std::shared_ptr<geometry_msgs::msg::Vector3Stamped> message = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
  message->header.stamp = msg->header.stamp;
  message->header.frame_id = msg->header.frame_id;
  // The conjugate angles have the opposite sign, so we negate them.
  message->vector.x = -angles[0];
  message->vector.y = -angles[1];
  message->vector.z = -angles[2];

  RCLCPP_INFO(this->get_logger(), "New quaternion goal received: x: '%.2f', y: '%.2f', z: '%.2f'", message->vector.x, message->vector.y, message->vector.z);
  goal_ = message;
}

void GremsyDriver::enableLockModeCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                          const std::shared_ptr<std_srvs::srv::SetBool::Response> response){
  int new_mode = request->data ? 1 : 2;
  // If requested mode is already active
  if (new_mode == gimbal_mode_){
    response->success = true;
    response->message = "Gimbal is already in requested mode.";
    RCLCPP_WARN(this->get_logger(), "Gimbal mode unchanged, is already in %s mode.", gimbal_mode_ == 1 ? "lock" : "follow");
  } else {
    // Set new mode internally and to parameters.
    gimbal_mode_ = new_mode;
    this->set_parameter(rclcpp::Parameter("gimbal_mode", gimbal_mode_));
    if (gimbal_mode_ == 1) {
      gimbal_interface_->set_gimbal_lock_mode_sync();
    } else {
      gimbal_interface_->set_gimbal_follow_mode_sync();
    }

    response->success = true;
    response->message = "Gimbal mode successfully changed.";

    RCLCPP_INFO(this->get_logger(), "Changing gimbal mode to %s.", gimbal_mode_ == 1 ? "lock" : "follow");
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
    "gimbal_mode", 2,
    getParamDescriptor(
      "gimbal_mode", "Control mode of the gimbal",
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, 0, 2));

  this->declare_parameter(
    "lock_yaw_to_vehicle", true,
    getParamDescriptor(
      "lock_yaw_to_vehicle",
      "Uses the yaw relative to the gimbal mount to prevent drift issues. Only a light stabilization is applied.",
      rcl_interfaces::msg::ParameterType::PARAMETER_BOOL));

}


} // namespace ros2_gremsy
