ARG ROS_DISTRO=rolling

FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get upgrade -y \
    && \
    : "remove cache" && \
    apt-get autoremove -y -qq && \
    rm -rf /var/lib/apt/lists/*

# Create workspace directory
RUN mkdir -p /home/ros2_ws/src

# Copy source code
COPY . /home/ros2_ws/src/ros2_control_demos

# Install dependencies
RUN cd /home/ros2_ws \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    # uncomment this line if you want to use the latest version of ros2_control
    # && vcs import src --input https://raw.githubusercontent.com/ros-controls/ros2_control_ci/master/ros_controls.rolling-on-$ROS_DISTRO.repos \
    && apt-get update \
    && rosdep install --from-paths src -i -y --rosdistro ${ROS_DISTRO} \
    --skip-keys ros-${ROS_DISTRO}-joint-state-publisher-gui --skip-keys ros-${ROS_DISTRO}-rviz2 \
    && \
    : "remove cache" && \
    apt-get autoremove -y -qq && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN cd /home/ros2_ws \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --cmake-args -DBUILD_TESTING=OFF --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

CMD ["ros2", "launch", "ros2_control_demo_example_7", "r6bot_controller.launch.py"]