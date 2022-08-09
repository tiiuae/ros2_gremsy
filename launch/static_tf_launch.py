from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    ld = LaunchDescription()

    # Environment variables
    DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID', "sad99")

    # Namespace declarations
    namespace = DRONE_DEVICE_ID

    # Frame names
    fcu_frame = DRONE_DEVICE_ID + "/fcu"
    gimbal_mount_frame = DRONE_DEVICE_ID + "/gimbal_mount"
    gimbal_frame = DRONE_DEVICE_ID + "/gimbal"

    # Node definitions
    ld.add_action(
        Node(
            namespace=namespace,
            package="tf2_ros",
            executable="static_transform_publisher",
            name="fcu_to_gimbal_static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", fcu_frame, gimbal_mount_frame],
            output="screen",
        ),
    )

    return ld
