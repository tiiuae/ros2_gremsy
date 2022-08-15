FROM ghcr.io/tiiuae/fog-ros-baseimage:builder-latest AS builder

COPY . /main_ws/src/

# this:
# 1) builds the application
# 2) packages the application as .deb in /main_ws/

RUN /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:stable

ENTRYPOINT exec ros-with-env ros2 launch ros2_gremsy ros_gremsy_gimbal.launch.py serial_port:=${GIMBAL_SERIAL_PORT}

COPY --from=builder /main_ws/ros-*-ros2-gremsy_*_amd64.deb /ros2_gremsy.deb

RUN apt update && apt install -y --no-install-recommends ./ros2_gremsy.deb \
	&& rm /ros2_gremsy.deb
