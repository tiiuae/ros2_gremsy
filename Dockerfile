FROM --platform=${BUILDPLATFORM:-linux/amd64} ghcr.io/tiiuae/fog-ros-sdk:v3.1.0-${TARGETARCH:-amd64} AS builder

# Must be defined another time after "FROM" keyword.
ARG TARGETARCH

# SRC_DIR environment variable is defined in the fog-ros-sdk image.
# The same workspace path is used by all ROS2 components.
# See: https://github.com/tiiuae/fog-ros-baseimage/blob/main/Dockerfile.sdk_builder
COPY . $SRC_DIR/ros2_gremsy

RUN /packaging/build_colcon_sdk.sh ${TARGETARCH:-amd64}
# Even though it is possible to tar the install directory for retrieving it later in runtime image,
# the tar extraction in arm64 emulated on arm64 is still slow. So, we copy the install directory instead

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:v3.1.0

ENTRYPOINT [ "/entrypoint.sh" ]

COPY entrypoint.sh /entrypoint.sh

# WORKSPACE_DIR environment variable is defined in the fog-ros-baseimage.
# The same installation directory is used by all ROS2 components.
# See: https://github.com/tiiuae/fog-ros-baseimage/blob/main/Dockerfile
WORKDIR $WORKSPACE_DIR
COPY --from=builder $WORKSPACE_DIR/install install
