FROM ros:rolling

# Set up the ROS 2 environment
ENV ROS_WS /ros_ws

COPY src/ $ROS_WS/src/

# Install build tools and dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    python3-venv \
    python3-full \
    python3-colcon-common-extensions \
    python3-opencv \
    python3-dev \
    python3-numpy \
    python3-numpy-dev \
    ros-rolling-cv-bridge \
    ros-rolling-sensor-msgs \
    git \
    adb \
    v4l-utils \
    v4l2loopback-dkms \
    v4l2loopback-utils \
    scrcpy \
    && rm -rf /var/lib/apt/lists/*

WORKDIR $ROS_WS

# Create virtual environment with --system-site-packages to use system numpy
ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv --system-site-packages $VIRTUAL_ENV
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

# Install Python dependencies in the virtual environment
RUN pip install --upgrade pip && \
    pip install setuptools wheel

# Link system numpy include dirs
RUN python3 -c "import numpy; print('System numpy include path:', numpy.get_include())" && \
    mkdir -p /usr/local/include/numpy && \
    cp -r /usr/include/numpy/* /usr/local/include/numpy/ || true && \
    echo 'export C_INCLUDE_PATH=/usr/include/numpy:$C_INCLUDE_PATH' >> ~/.bashrc && \
    echo 'export CPLUS_INCLUDE_PATH=/usr/include/numpy:$CPLUS_INCLUDE_PATH' >> ~/.bashrc

# Install ultralytics and related dependencies
RUN pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu && \
    pip install ultralytics

# Modify setup.py to add ultralytics dependency
RUN sed -i 's/install_requires=\[[^]]*\]/install_requires=["setuptools", "ultralytics"]/' $ROS_WS/src/scrcpy_ros/setup.py || \
    echo "无法修改setup.py，请检查文件路径"

# Install dependencies from requirements.txt
RUN if [ -f "$ROS_WS/src/scrcpy_ros/requirements.txt" ]; then \
    sed -i 's/numpy==1.21.5/numpy/g' $ROS_WS/src/scrcpy_ros/requirements.txt && \
    grep -v "numpy" $ROS_WS/src/scrcpy_ros/requirements.txt > $ROS_WS/src/scrcpy_ros/requirements.txt.tmp && \
    mv $ROS_WS/src/scrcpy_ros/requirements.txt.tmp $ROS_WS/src/scrcpy_ros/requirements.txt && \
    pip install -r $ROS_WS/src/scrcpy_ros/requirements.txt || echo "部分依赖安装失败，但将继续构建"; \
    fi

# Pre-download YOLO model
RUN mkdir -p /root/.config/Ultralytics && \
    (python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')" || \
    echo "模型下载将在运行时尝试")

# Build ROS packages
RUN . /opt/ros/rolling/setup.sh && \
    colcon build --symlink-install || echo "ROS构建部分失败，但将继续"

# Set environment variables
ENV PYTHONPATH=$PYTHONPATH:$VIRTUAL_ENV/lib/python3.12/site-packages:$ROS_WS/install/lib/python3.12/site-packages

# Automatically load ROS and virtual environment on container start
RUN echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc && \
    echo "source $ROS_WS/install/setup.bash" >> ~/.bashrc && \
    echo "source $VIRTUAL_ENV/bin/activate" >> ~/.bashrc && \
    echo "export PYTHONPATH=\$PYTHONPATH:$VIRTUAL_ENV/lib/python3.12/site-packages:$ROS_WS/install/lib/python3.12/site-packages" >> ~/.bashrc

WORKDIR $ROS_WS

# Keep the container running
CMD ["bash"]