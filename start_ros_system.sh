#!/bin/bash

# 解析命令行参数
WITH_DOCKER=false
USE_SCRCPY=false

for arg in "$@"
do
    case $arg in
        --with-docker)
        WITH_DOCKER=true
        shift
        ;;
        --with-scrcpy)
        USE_SCRCPY=true
        shift
        ;;
    esac
done

if [ "$WITH_DOCKER" = true ]; then
    # 使用run.sh启动Docker环境
    ./run.sh
else
    # 本地启动ROS环境
    echo "本地启动ROS环境..."
    
    # 设置ROS环境变量
    ROS_FOUND=false
    if [ -f /opt/ros/rolling/setup.bash ]; then
        source /opt/ros/rolling/setup.bash
        echo "已载入ROS Rolling环境"
        ROS_FOUND=true
    else
        echo "警告: 未找到ROS Rolling环境文件，可能需要手动设置环境"
        # 尝试查找其他可能的ROS版本
        ROS_VERSIONS=("humble" "iron" "foxy" "galactic")
        for version in "${ROS_VERSIONS[@]}"; do
            if [ -f "/opt/ros/$version/setup.bash" ]; then
                source "/opt/ros/$version/setup.bash"
                echo "已载入ROS $version 环境"
                ROS_FOUND=true
                break
            fi
        done
    fi
    
    # 如果需要使用scrcpy连接到手机
    if [ "$USE_SCRCPY" = true ]; then
        # 检查scrcpy是否安装
        if ! command -v scrcpy &> /dev/null; then
            echo "错误: scrcpy未安装，请先安装scrcpy"
            exit 1
        fi
        
        echo "启动scrcpy连接到手机屏幕..."
        # 获取scrcpy版本并根据版本使用合适的参数
        SCRCPY_VERSION=$(scrcpy --version 2>&1 | grep -oP 'scrcpy \K[0-9.]+' || echo "unknown")
        echo "检测到scrcpy版本: $SCRCPY_VERSION"
        
        # 根据版本使用不同的命令行参数
        if [ "$SCRCPY_VERSION" = "unknown" ]; then
            # 使用最基本的参数，大多数版本都支持
            scrcpy &
        else
            # 1.25版本不支持 --no-audio，但支持其他参数
            scrcpy --always-on-top &
        fi
    fi
    
    # 进入bash交互环境
    exec bash
fi
