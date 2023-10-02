FROM ghcr.io/tiiuae/fog-ros-baseimage-builder:v3.0.0 AS builder

COPY . /main_ws/src/

# this:
# 1) builds the application
# 2) packages the application as .deb in /main_ws/

RUN /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:v2.1.0

ENTRYPOINT exec ros-with-env ros2 run ros2_gremsy gremsy_node --ros-args --remap __ns:=/$DRONE_DEVICE_ID -p com_port:=/dev/ttyUSB0

COPY --from=builder /main_ws/ros-*-ros2-gremsy_*_amd64.deb /ros2_gremsy.deb

RUN apt update && apt install -y --no-install-recommends ./ros2_gremsy.deb \
	&& rm /ros2_gremsy.deb
