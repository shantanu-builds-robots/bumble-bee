# Build/test environment for this workspace: ROS 2 Humble + Gazebo Classic 11.
#
#   docker build -t bumble-bee:humble docker/
#   docker run --rm -it -v "$PWD":/ws/src/drone_delivery bumble-bee:humble \
#     bash -c "cd /ws && colcon build && source install/setup.bash && bash"
FROM osrf/ros:humble-desktop-full
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
      ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro \
      ros-humble-cv-bridge ros-humble-tf2-geometry-msgs ros-humble-tf2-kdl \
    && rm -rf /var/lib/apt/lists/*
