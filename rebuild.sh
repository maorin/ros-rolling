#!/bin/bash

echo "从头开始构建Docker镜像（不使用缓存）..."
docker-compose build --no-cache

echo "构建完成。" 