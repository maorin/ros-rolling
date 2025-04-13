#!/bin/bash

# 检查是否是SSH会话
IS_SSH=false
if [ -n "$SSH_CLIENT" ] || [ -n "$SSH_TTY" ]; then
    IS_SSH=true
    echo "检测到SSH会话..."
fi

# 确保脚本可执行
chmod +x build.sh

# 如果容器不存在，先构建容器
if [ ! "$(docker ps -a | grep ros_rolling_dev)" ]; then
    echo "构建Docker容器..."
    ./build.sh
fi

# 如果容器没有运行，启动容器
if [ ! "$(docker ps -q -f name=ros_rolling_dev)" ]; then
    echo "启动Docker容器..."
    docker-compose up -d
fi

# 进入容器的交互式bash终端
echo "进入ROS开发环境..."

# 如果是SSH会话，确保使用正确的终端分配参数
if [ "$IS_SSH" = true ]; then
    # 添加 -t 参数确保终端正确分配
    docker exec -it ros_rolling_dev bash
else
    docker exec -it ros_rolling_dev bash
fi

# 提供一个退出消息
echo "已退出ROS环境" 