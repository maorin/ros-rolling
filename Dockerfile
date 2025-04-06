FROM ros:rolling

# Set up the ROS 2 environment
ENV ROS_WS=/ros_ws

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
    # VNC and NoVNC dependencies
    tigervnc-standalone-server \
    tigervnc-xorg-extension \
    novnc \
    openbox \
    xterm \
    lxpanel \
    pcmanfm \
    x11-xserver-utils \
    xfonts-base \
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
ENV PYTHONPATH="$VIRTUAL_ENV/lib/python3.12/site-packages:$ROS_WS/install/lib/python3.12/site-packages"

# Automatically load ROS and virtual environment on container start
RUN echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc && \
    echo "source $ROS_WS/install/setup.bash" >> ~/.bashrc && \
    echo "source $VIRTUAL_ENV/bin/activate" >> ~/.bashrc && \
    echo "export PYTHONPATH=\$PYTHONPATH:$VIRTUAL_ENV/lib/python3.12/site-packages:$ROS_WS/install/lib/python3.12/site-packages" >> ~/.bashrc

# Setup VNC password and configuration
RUN mkdir -p /root/.vnc
RUN echo "ros" | vncpasswd -f > /root/.vnc/passwd && chmod 600 /root/.vnc/passwd
RUN mkdir -p /root/.config/openbox
RUN echo '#!/bin/bash\nlxpanel &\npcmanfm --desktop &' > /root/.config/openbox/autostart && chmod +x /root/.config/openbox/autostart

# Create VNC startup script
RUN echo '#!/bin/bash\nVNC_DISPLAY=1\nNOVNC_PORT=6080\nVNC_PORT=590${VNC_DISPLAY}\nvncserver :${VNC_DISPLAY} -geometry 1280x720 -depth 24 -localhost no\n/usr/share/novnc/utils/novnc_proxy --vnc localhost:${VNC_PORT} --listen ${NOVNC_PORT}' > /start_vnc.sh && chmod +x /start_vnc.sh

WORKDIR $ROS_WS

# Expose NoVNC web port
EXPOSE 6080

# Create a new entrypoint script
RUN echo '#!/bin/bash\n\n# Start VNC in background\n/start_vnc.sh &\n\n# Keep the container running\nexec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash", "-c", "while true; do sleep 1; done"]