#!/bin/bash

# 停止并移除运行中的Docker容器
echo "停止ROS Docker开发环境..."
docker-compose down

echo "ROS Docker开发环境已停止。" 