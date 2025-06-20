FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV CMAKE_PREFIX_PATH=/opt/ros/jazzy:$CMAKE_PREFIX_PATH

# Install all system and ROS 2 dependencies
RUN apt update && apt install -y \
    python3-pip \
    python3-dev \
    build-essential \
    libatlas-base-dev \
    git \
    python3-colcon-common-extensions \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-control-cmake \
    ros-jazzy-ros2-control-test-assets \
    ros-jazzy-hardware-interface \
    ros-jazzy-controller-interface \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joy \
    ros-jazzy-test-msgs \   
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Just install Python dependencies you need
RUN python3 -m pip install --break-system-packages numpy cython

# Setup workspace
WORKDIR /ros2_ws
COPY . ./src

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

CMD ["/bin/bash"]
