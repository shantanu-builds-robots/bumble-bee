# Bumble Bee: ROS 2 Humble + Gazebo Classic 11, with this repository built in /ws.
#
#   docker build -f docker/Dockerfile -t bumble-bee .
#
# See "Run with Docker" in the README for running it with Gazebo's window on your desktop.
FROM osrf/ros:humble-desktop-full
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /ws

# Packages the manifests do not declare.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
      ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro \
      ros-humble-cv-bridge ros-humble-tf2-geometry-msgs ros-humble-tf2-kdl \
    && rm -rf /var/lib/apt/lists/*

# Dependencies first, from the package manifests only, so editing code does not redo this layer.
COPY drone_delivery/package.xml src/drone_delivery/drone_delivery/package.xml
COPY drone_plugin/package.xml src/drone_delivery/drone_plugin/package.xml
RUN apt-get update && rosdep update --rosdistro humble \
    && rosdep install --from-paths src --ignore-src -y --rosdistro humble \
    && rm -rf /var/lib/apt/lists/*

COPY . src/drone_delivery
RUN source /opt/ros/humble/setup.bash && colcon build && rm -rf build log

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && echo 'source /ws/install/setup.bash' >> /root/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
