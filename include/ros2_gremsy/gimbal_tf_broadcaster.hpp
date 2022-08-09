
#ifndef GIMBAL_TF_BROADCASTER_HPP
#define GIMBAL_TF_BROADCASTER_HPP

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include <memory>
#include <string>

using std::placeholders::_1;

namespace ros2_gremsy
{

/**
 * @class ros2_gremsy::GimbalFramePublisher
 * @brief Transform publisher for gremsy gimbal
 */
class GimbalFramePublisher : public rclcpp::Node
{
public:
  GimbalFramePublisher();
  ~GimbalFramePublisher();

private:
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr orientation_sub_;

  void orientationCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg);

  // tf
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // tf frames
  std::string gimbal_frame_, gimbal_mount_frame_;

  std::string orientation_topic_;
};

} // namespace ros2_gremsy

#endif // GIMBAL_TF_BROADCASTER_HPP