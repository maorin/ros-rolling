FROM ros:rolling

# Set up the ROS 2 environment
ENV ROS_WS /ros_ws

COPY src/ $ROS_WS/src/

# Install build tools and dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-opencv \
    ros-rolling-cv-bridge \
    ros-rolling-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR $ROS_WS

RUN pip3 install -r $ROS_WS/src/scrcpy_ros/requirements.txt || true

RUN . /opt/ros/rolling/setup.sh && \
    colcon build --symlink-install

# Source the ROS 2 environment when running a container
RUN echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc && \
    echo "source $ROS_WS/install/setup.bash" >> ~/.bashrc

WORKDIR $ROS_WS

# Keep the container running
CMD ["bash"]