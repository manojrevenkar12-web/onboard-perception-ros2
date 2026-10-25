# syntax=docker/dockerfile:1
# One image for every role (camera, detector, tracking), amd64 or arm64 (Jetson Orin, CPU path).
#   docker build -f docker/Dockerfile -t onboard-perception-ros2 .
#   docker buildx build --platform linux/arm64 -f docker/Dockerfile -t onboard-perception-ros2:arm64 .
ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base AS build
ARG ROS_DISTRO
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR /ws
COPY src/ src/
RUN apt-get update \
 && rosdep update --rosdistro "${ROS_DISTRO}" \
 && rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO}" \
 && rm -rf /var/lib/apt/lists/*
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && colcon build --merge-install --install-base /opt/opr \
      --packages-up-to opr_pipeline opr_bench \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ros:${ROS_DISTRO}-ros-base AS runtime
ARG ROS_DISTRO
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY --from=build /ws/src /tmp/src
RUN apt-get update \
 && rosdep update --rosdistro "${ROS_DISTRO}" \
 && rosdep install --from-paths /tmp/src --ignore-src -y --rosdistro "${ROS_DISTRO}" --dependency-types exec \
 && rm -rf /var/lib/apt/lists/* /tmp/src
COPY --from=build /opt/opr /opt/opr
# The camera needs /dev/videoN: run with --device and the video group, not as root.
RUN useradd --create-home --groups video --uid 1001 opr
USER opr
ENV ROS_DISTRO=${ROS_DISTRO}
COPY --chmod=755 docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "opr_pipeline", "pipeline.launch.py"]
