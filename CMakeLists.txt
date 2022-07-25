cmake_minimum_required(VERSION 3.8)
project(ros2_gremsy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

include_directories(
        gSDK/src/mavlink/include/mavlink/v2.0
        gSDK/src/
)

set(SOURCES src/gremsy.cpp gSDK/src/serial_port.cpp gSDK/src/gimbal_interface.cpp)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(gremsy ${SOURCES})
#target_include_directories(gremsy PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#  ${CMAKE_SOURCE_DIR}/gSDK/src)

ament_target_dependencies(gremsy PUBLIC rclcpp std_msgs std_srvs sensor_msgs geometry_msgs tf2 tf2_geometry_msgs Eigen3 builtin_interfaces)


# DepthAI GStreamer as separate node
add_executable(gremsy_node src/gremsy_node.cpp)
ament_target_dependencies(gremsy_node PUBLIC rclcpp rclcpp_components)
target_link_libraries(gremsy_node PUBLIC gremsy)

install(TARGETS gremsy gremsy_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
