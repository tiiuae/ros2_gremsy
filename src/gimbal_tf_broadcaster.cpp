

#include "ros2_gremsy/gimbal_tf_broadcaster.hpp"

namespace ros2_gremsy
{

GimbalFramePublisher::GimbalFramePublisher():
Node("gimbal_tf2_frame_publisher")
{

  declare_parameter(
    "gimbal_mount_frame",
    rclcpp::ParameterValue(std::string("gimbal_mount_frame")));
  declare_parameter(
    "gimbal_frame",
    rclcpp::ParameterValue(std::string("gimbal_frame")));
  declare_parameter(
    "orientation_topic_name",
    rclcpp::ParameterValue(std::string("~/mount_orientation_local")));

  gimbal_mount_frame_ = get_parameter("gimbal_mount_frame").as_string();
  gimbal_frame_ = get_parameter("gimbal_frame").as_string();
  orientation_topic_ = get_parameter("orientation_topic_name").as_string();

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  orientation_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
    orientation_topic_, 10,
    std::bind(&GimbalFramePublisher::orientationCallback, this, _1));
}

GimbalFramePublisher::~GimbalFramePublisher()
{
}


void
GimbalFramePublisher::orientationCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped tf;

  tf.header.stamp = this->get_clock()->now();
  tf.header.frame_id = gimbal_mount_frame_;
  tf.child_frame_id = gimbal_frame_;
  tf.transform.translation.x = 0;
  tf.transform.translation.y = 0;
  tf.transform.translation.z = -0.05;
  tf.transform.rotation.w = msg->quaternion.w;
  tf.transform.rotation.x = msg->quaternion.x;
  tf.transform.rotation.y = msg->quaternion.y;
  tf.transform.rotation.z = msg->quaternion.z;

  tf_broadcaster_->sendTransform(tf);
}

} // namespace ros2_gremsy

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_gremsy::GimbalFramePublisher>());
  rclcpp::shutdown();
  return 0;
}
