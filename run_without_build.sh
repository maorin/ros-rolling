#!/bin/bash

# 该脚本用于在不重新构建的情况下运行容器，仅拉取已构建的镜像并运行

echo "启动已构建的Docker容器..."

# 如果容器已在运行，则直接进入
if [ "$(docker ps -q -f name=ros_rolling_dev)" ]; then
  echo "容器已在运行，直接进入..."
  docker exec -it ros_rolling_dev bash
  exit 0
fi

# 如果容器存在但未运行，则启动它
if [ "$(docker ps -aq -f name=ros_rolling_dev)" ]; then
  echo "容器存在但未运行，启动容器..."
  docker start ros_rolling_dev
  docker exec -it ros_rolling_dev bash
  exit 0
fi

# 如果没有容器，则使用docker-compose启动
echo "没有找到现有容器，使用docker-compose启动..."
docker-compose up -d
docker exec -it ros_rolling_dev bash 