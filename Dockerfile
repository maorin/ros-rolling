FROM ros:foxy

# Set up the ROS 2 environment
ENV ROS_WS /ros2_ws

COPY src/ $ROS_WS/src

# Install build tools and dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR $ROS_WS

RUN . /opt/ros/foxy/setup.sh && \
    colcon build --symlink-install

# Source the ROS 2 environment when running a container
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc