FROM ros:jazzy-ros-base

# Install basic tools and colcon
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    ros-jazzy-ros2-control \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joy \
    && rm -rf /var/lib/apt/lists/*

# Copy your code into the container
COPY . /ros2_ws/src/ros2_control_r6bot

# Build the workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /ros2_ws && \
    colcon build --symlink-install"

# Source the workspace at container start
CMD ["/bin/bash"]
