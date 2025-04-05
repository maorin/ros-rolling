# ros-foxy
# ros-rolling

# 编译
colcon build
source install/setup.bash

# 从手机上获取屏幕 把检测到的游戏画面发布到ros2
ros2 run scrcpy_ros scrcpy_publisher

# 把检测到的人 进行单目标跟踪  把最终单目标 发布到ros2
ros2 run scrcpy_ros tracks_sort

# 自动瞄准
ros2 run scrcpy_ros auto_aim

# ROS Docker 开发环境

这是一个使用 Docker 在 MacBook ARM 架构上开发 ROS Rolling 的项目。

## 前提条件

- 安装 [Docker Desktop for Mac](https://www.docker.com/products/docker-desktop/)
- 如需图形界面支持，安装 [XQuartz](https://www.xquartz.org/) 并启用"允许来自网络客户端的连接"选项

## 使用方法

1. 克隆此仓库到您的本地机器
2. 使用以下命令启动 Docker 开发环境：

```bash
# 赋予脚本执行权限
chmod +x build.sh start_ros_system.sh

# 启动 ROS Docker 开发环境
./start_ros_system.sh
```

3. 在容器内，您可以使用标准 ROS 命令进行开发和测试：

```bash
# 构建工作空间
colcon build

# 加载环境
source install/setup.bash

# 运行您的 ROS 节点
ros2 run scrcpy_ros scrcpy_publisher
```

## 项目结构

- `src/` - ROS 源代码目录，与主机共享
- `Dockerfile` - Docker 镜像定义
- `docker-compose.yml` - Docker 容器配置
- `build.sh` - 用于构建 Docker 镜像的脚本
- `start_ros_system.sh` - 用于启动开发环境的脚本
