#!/bin/bash

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
docker exec -it ros_rolling_dev bash
