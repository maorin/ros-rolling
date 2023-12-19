#!/bin/bash

# 终止脚本，如果任何命令返回非零状态
set -e

# 构建ROS工作空间
colcon build

# 设置环境变量
#source install/setup.bash
. install/setup.bash

# 启动scrcpy_ros的scrcpy_publisher节点
nohup ros2 run scrcpy_ros scrcpy_publisher &

# 启动scrcpy_ros的tracks_sort节点
nohup ros2 run scrcpy_ros tracks_sort &

# 可以在这里添加更多的ROS节点或其他命令

echo "ROS系统启动完成。"
