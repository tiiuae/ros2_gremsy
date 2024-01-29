from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os


def generate_launch_description():
    ld = LaunchDescription()

    # Environment variables
    DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID', "sad99")

    # Namespace declarations
    namespace = DRONE_DEVICE_ID

    # Arguments
    ld.add_action(DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"))

    # Frame names
    fcu_frame = DRONE_DEVICE_ID + "/fcu"
    gimbal_mount_frame = DRONE_DEVICE_ID + "/gimbal_mount"
    gimbal_frame = DRONE_DEVICE_ID + "/gimbal"

    ld.add_action(
        Node(
            namespace=namespace,
            package='ros2_gremsy',
            executable='gremsy_node',
            name='ros2_gremsy',
            parameters=[{
                'com_port': LaunchConfiguration("serial_port"),

            }],
            output='screen',
            remappings=[
                ('ros2_gremsy/gimbal_goal', '/external/ros2_gremsy/gimbal_goal'),
            ],
        ),
    ),

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/static_tf_launch.py'])
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/tf_broadcaster_launch.py'])
        )
    )

    return ld
