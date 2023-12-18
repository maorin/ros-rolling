colcon build
source install/setup.bash

nohup ros2 run scrcpy_ros scrcpy_publisher &

nohup ros2 run scrcpy_ros tracks_sort &