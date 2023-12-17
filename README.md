# ros-foxy
# ros-rolling

# 编译
colcon build
source install/setup.bash

# 从手机上获取屏幕 把检测到的人脸信息发布到ros2
ros2 run scrcpy_ros scrcpy_publisher

# 把检测到的人 进行单目标跟踪  把最终单目标 发布到ros2
ros2 run scrcpy_ros tracks_sort

# 自动瞄准
ros2 run scrcpy_ros auto_aim