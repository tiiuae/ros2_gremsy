# ros2_gremsy: gSDK based Gremsy Gimbal Driver Node for ROS2

A ROS2 interface to control Gremsy gimbals. Based on the [Gremsy/gSDK](https://github.com/Gremsy/gSDK) interface, [Flova/ros_gremsy](https://github.com/Flova/ros_gremsy)
, and the MavLink protocol. 

Disclaimer: This software package is not officially developed by or related to Gremsy.

## Description
This package utilizes UART communication with gimbal COM port to control the gimbal.

The Gremsy gimbals using MavLink protocol for message format and gSDK provides necessary functions for communication. 

This package is only tested with Gremsy MIO and ROS2 Galactic so far. Feel free to contribute if you have another model working with this package, or if there are some modifications needed for it.

## Setup

Clone this repository to `src` folder of your workspace. `--recurse-submodules` flag will automatically clone the required gSDK submodule.

```
git clone --recurse-submodules https://github.com/tiiuae/ros2_gremsy
```

Install dependencies using rosdep from the workspace directory, modify ${ROS_DISTRO} if you have in your environment.

```
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
```

Run the colcon to build package.
```
colcon build --symlink-install
```

Check the device name of the gimbal by running the following command.
```
dmesg | grep tty
```

The default com_port parameter is already `/dev/ttyUSB0`. If the device name is different, you should add it to run command. The launch file is not created yet, so you should manually enter it to the command.
```
ros2 run ros2_gremsy gremsy_node --ros-args -p com_port:=/dev/ttyUSB0
```

## Run with docker image
The default com_port parameter is already `/dev/ttyUSB0`. If the device name is different, you should use the correct one to mount the device. For example, `--device /dev/ttyUSB1:/dev/ttyUSB0`, so host `ttyUSB1` is mounted to container as `ttyUSB0`.

```
docker run --rm -ti --env DRONE_DEVICE_ID=sad99 --network host --device /dev/ttyUSB0:/dev/ttyUSB0 ghcr.io/tiiuae/tii-gremsy:main
```

## Cannot open serial port error
By default, the ubuntu user does not have access to the serial port. You can add it by 
```
sudo usermod -a -G tty ${USER}
sudo usermod -a -G dialout ${USER}
```
Reboot the computer.


## Published Topics
| Topic name  | Type | Description |
|-----|----|----|
| ~/imu | sensor_msgs/Imu | IMU data |
| ~/encoder | geometry_msgs/Vector3Stamped | Encoder data |
| ~/mount_orientation_global | geometry_msgs/QuaternionStamped | Orientation of the gimbal in the global frame |
| ~/mount_orientation_local | geometry_msgs/QuaternionStamped | Orientation of the gimbal in the local frame |

## Subscribed Topics
| Topic name  | Type | Description |
|-----|----|----|
| ~/gimbal_goal | geometry_msgs/Vector3Stamped | Goal orientation of the gimbal in the global frame in radians. X->Roll, Y->Pitch, Z->Yaw |
| ~/gimbal_goal_quaternion | geometry_msgs/QuaternionStamped | Goal orientation of the gimbal in the local frame as quaternion. |

## Services
| Service name | Service type     | Input type | Output types                 | Description                                                  |
|--------------|------------------|------------|------------------------------|--------------------------------------------------------------|
| ~/lock_mode  | std_srvs/SetBool | bool data  | bool success, string message | Change gimbal mode: lock mode (true) and follow mode (false) |

## Parameters

| Parameter name  | Type | Description | Accepted values| Default value | 
|----|----|----|----|----|
|device_id|integer|Device id- 0: MIO, 1: S1, 2: T3V3, 3: T7|0,1,2,3|0|
|com_port|string|Serial device for the gimbal connection|-|/dev/ttyUSB0|
|baudrate|integer|Baudrate for the gimbal connection|-|115200|
|state_poll_rate|double|Rate in which the gimbal data is polled and published|0.0-300.0|50.0|
|goal_push_rate|double|Rate in which the gimbal are pushed to the gimbal|0.0-300.0|60.0|
|gimbal_mode|integer|Control mode of the gimbal 0:GIMBAL_OFF, 1:LOCK_MODE, 2:FOLLOW_MODE|0,1,2|1|
|tilt_axis_input_mode|integer|Input mode of the gimbals tilt, 0:CTRL_ANGLE_BODY_FRAME, 1: CTRL_ANGULAR_RATE, 2:CTRL_ANGLE_ABSOLUTE_FRAME|0,1,2|2|
|tilt_axis_stabilize|boolean|Input mode of the gimbals tilt|-|true|
|roll_axis_input_mode|integer|Input mode of the gimbals roll, 0:CTRL_ANGLE_BODY_FRAME, 1:CTRL_ANGULAR_RATE, 2:CTRL_ANGLE_ABSOLUTE_FRAME|0,1,2|2|
|roll_axis_stabilize|boolean|Input mode of the gimbals roll|-|true|
|pan_axis_input_mode|integer|Input mode of the gimbals pan, 0:CTRL_ANGLE_BODY_FRAME, 1:CTRL_ANGULAR_RATE, 2:CTRL_ANGLE_ABSOLUTE_FRAME|0,1,2|2|
|pan_axis_stabilize|boolean|Input mode of the gimbals pan|-|true|
|lock_yaw_to_vehicle|boolean|Uses the yaw relative to the gimbal mount to prevent drift issues. Only a light stabilization is applied.|-|true|

Note: Only Gimbal Pixy and T3V3 support CTRL_ANGLE_BODY_FRAME mode with pitch and yaw axis.

# TODO:
- Create a launch file and parameters file for the package.
- Verify other models working with this package
- Add TF publishing 
- Add other topics for controlling gimbal.

# References
[gSDK](https://github.com/Gremsy/gSDK) 
[Flova/ros_gremsy](https://github.com/Flova/ros_gremsy)
