FROM ghcr.io/tiiuae/fog-ros-baseimage:builder-7072ebc AS builder

COPY . /main_ws/src/

# this:
# 1) builds the application
# 2) packages the application as .deb in /main_ws/

RUN /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-7072ebc

ENTRYPOINT ["/entrypoint.sh"]

COPY entrypoint.sh /entrypoint.sh

COPY --from=builder /main_ws/ros-*-ros2-gremsy_*_amd64.deb /ros2_gremsy.deb

RUN apt update && apt install -y --no-install-recommends ./ros2_gremsy.deb \
	&& rm /ros2_gremsy.deb
