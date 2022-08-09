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

    ld.add_action(
        Node(
            namespace=namespace,
            package="ros2_gremsy",
            executable="gimbal_tf_broadcaster",
            name="gimbal_tf_broadcaster",
            parameters=[
                {"gimbal_mount_frame": gimbal_mount_frame,
                 "gimbal_frame": gimbal_frame,
                 "orientation_topic_name": "/{:s}/ros2_gremsy/mount_orientation_local".format(DRONE_DEVICE_ID)}
            ]
        ),
    )

    return ld
