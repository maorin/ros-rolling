#!/bin/bash

# 此脚本直接在本地机器上启动ROS环境并连接手机屏幕，专为SSH远程访问设计
# 会在远程机器的物理显示屏上显示scrcpy界面

# 检测是否通过SSH连接
IS_SSH=false
if [ -n "$SSH_CLIENT" ] || [ -n "$SSH_TTY" ]; then
    IS_SSH=true
    echo "检测到SSH会话..."
fi

# 设置ROS环境变量
ROS_FOUND=false
if [ -f /opt/ros/rolling/setup.bash ]; then
    source /opt/ros/rolling/setup.bash
    echo "已载入ROS Rolling环境"
    ROS_FOUND=true
else
    echo "警告: 未找到ROS Rolling环境文件"
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

if [ "$ROS_FOUND" = false ]; then
    echo "警告: 未找到任何ROS环境。ROS命令可能无法使用。"
    
    # 检查 /opt/ros 目录是否存在，如果存在则列出内容
    if [ -d "/opt/ros" ]; then
        echo "可用的ROS安装目录:"
        ls -l /opt/ros/
    else
        echo "未找到 /opt/ros 目录，请确认ROS已正确安装"
    fi
fi

# 检查scrcpy是否安装
if ! command -v scrcpy &> /dev/null; then
    echo "错误: scrcpy未安装，请先安装scrcpy"
    echo "Ubuntu/Debian安装命令: sudo apt install scrcpy"
    exit 1
fi

# 查找连接的Android设备
echo "检查连接的Android设备..."
adb devices -l

# 如果是SSH会话，需要设置DISPLAY变量指向物理显示器
if [ "$IS_SSH" = true ]; then
    echo "SSH会话中，设置显示到远程机器的物理屏幕..."
    
    # 查找活动的X会话DISPLAY
    ACTIVE_DISPLAY=$(w -hs | grep -v "pts/" | grep -o ":[0-9]\+" | head -1)
    
    if [ -z "$ACTIVE_DISPLAY" ]; then
        # 如果找不到活动会话，尝试使用默认的:0
        ACTIVE_DISPLAY=":0"
    fi
    
    # 检查该显示是否可用
    if ! xdpyinfo -display $ACTIVE_DISPLAY &>/dev/null; then
        echo "警告: 无法连接到显示 $ACTIVE_DISPLAY，尝试查找其他可用显示..."
        # 尝试常见的显示设置
        for disp in ":0" ":1" ":0.0"; do
            if xdpyinfo -display $disp &>/dev/null; then
                ACTIVE_DISPLAY=$disp
                echo "找到可用显示: $ACTIVE_DISPLAY"
                break
            fi
        done
    fi
    
    # 检查是否找到有效的显示
    if ! xdpyinfo -display $ACTIVE_DISPLAY &>/dev/null; then
        echo "错误: 无法找到有效的物理显示器。请确保远程机器有活动的图形界面。"
        
        # 提供诊断信息
        echo "---诊断信息---"
        echo "当前登录用户:"
        w
        echo "X服务器进程:"
        ps aux | grep X
        
        read -p "是否尝试强制使用:0显示? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            ACTIVE_DISPLAY=":0"
            echo "强制使用显示 $ACTIVE_DISPLAY"
        else
            echo "操作已取消。"
            exit 1
        fi
    fi
    
    # 设置DISPLAY环境变量
    export DISPLAY=$ACTIVE_DISPLAY
    export XAUTHORITY=$(find /home -name .Xauthority 2>/dev/null | head -1)
    
    echo "将在远程物理显示器 $DISPLAY 上显示scrcpy界面"
    
    # 验证访问权限
    touch /tmp/x11_access_test
    if ! xsetroot -cursor /tmp/x11_access_test 2>/dev/null; then
        echo "警告: 没有X服务器访问权限，尝试修正..."
        
        # 尝试设置访问权限
        DISPLAY_USER=$(who | grep "$ACTIVE_DISPLAY" | awk '{print $1}')
        if [ -n "$DISPLAY_USER" ]; then
            echo "显示器 $ACTIVE_DISPLAY 属于用户 $DISPLAY_USER"
            if [ "$DISPLAY_USER" != "$(whoami)" ]; then
                echo "您当前以 $(whoami) 身份运行，这可能导致权限问题"
                echo "考虑以下解决方案:"
                echo "1. 在远程机器上运行: xhost +LOCAL:"
                echo "2. 或者使用 'sudo -u $DISPLAY_USER ./local_ros_with_scrcpy.sh'"
            fi
        fi
    fi
    rm -f /tmp/x11_access_test
fi

# 启动scrcpy连接到手机屏幕
echo "启动scrcpy连接到手机屏幕..."

# 获取scrcpy版本并根据版本使用合适的参数
SCRCPY_VERSION=$(scrcpy --version 2>&1 | grep -oP 'scrcpy \K[0-9.]+' || echo "unknown")
echo "检测到scrcpy版本: $SCRCPY_VERSION"

# 根据版本使用不同的命令行参数
if [ "$SCRCPY_VERSION" = "unknown" ]; then
    # 使用最基本的参数，大多数版本都支持
    scrcpy --window-title="SCRCPY $DISPLAY" &
else
    # 使用兼容的参数，并设置合适的位置和大小
    scrcpy --always-on-top --window-title="SCRCPY $DISPLAY" --window-x 100 --window-y 100 --window-width 800 --window-height 600 &
fi

SCRCPY_PID=$!

# 显示一些有用的提示信息
echo ""
echo "=== ROS环境已启动 ==="
echo "提示:"
echo "- scrcpy已在远程机器物理显示器上启动，进程ID: $SCRCPY_PID"
if [ "$ROS_FOUND" = true ]; then
    echo "- 使用 'ros2 topic list' 查看当前话题"
    echo "- 使用 'ros2 node list' 查看当前节点"
fi
echo "- 退出时会保持scrcpy连接，使用 'pkill scrcpy' 关闭scrcpy"
echo "- 如果看不到scrcpy界面，检查远程机器的屏幕"
echo "- 权限问题解决方案:"
echo "  1. 在远程机器上运行: xhost +LOCAL:"
echo "  2. 或以显示拥有者运行: sudo -u 显示拥有者 $0"
echo "====================" 
echo ""

# 启动一个新的终端会话
exec bash 