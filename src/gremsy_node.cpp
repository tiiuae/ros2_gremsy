
#include "ros2_gremsy/gremsy.hpp"
using namespace ros2_gremsy;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::cout << "ROS2 Gremsy driver node." << std::endl;
    rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto gremsyDriver = std::make_shared<GremsyDriver>(options, "/dev/ttyUSB0");
    exec.add_node(gremsyDriver);
    exec.spin();

    rclcpp::shutdown();

    return 0;
}

