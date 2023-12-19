#!/bin/bash

# 杀死特定的ROS进程
# 你可以使用'ps'命令来查找进程的PID，然后使用kill命令终止它们
# 例如，杀死scrcpy_publisher和tracks_sort进程

kill $(ps aux | grep '/usr/bin/python3 /home/maojj/project/ros-rolling/install/scrcpy_ros/lib/scrcpy_ros/tracks_sort' | grep -v grep | awk '{print $2}')
kill $(ps aux | grep '/usr/bin/python3 /home/maojj/project/ros-rolling/install/scrcpy_ros/lib/scrcpy_ros/scrcpy_publisher' | grep -v grep | awk '{print $2}')

# 可以在这里添加更多的kill命令来终止其他进程

echo "ROS系统进程已清理完成。"

