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

这是一个使用 Docker 在 MacBook ARM 架构上开发 ROS Rolling 的项目。项目使用 YOLOv8 进行目标检测。

## 功能

- 从手机上获取屏幕，检测游戏画面中的物体并发布到ROS 2
  ```bash
  ros2 run scrcpy_ros scrcpy_publisher
  ```

- 对检测到的人进行单目标跟踪，把最终单目标发布到ROS 2
  ```bash
  ros2 run scrcpy_ros tracks_sort
  ```

- 自动瞄准
  ```bash
  ros2 run scrcpy_ros auto_aim
  ```

## 前提条件

- 安装 [Docker Desktop for Mac](https://www.docker.com/products/docker-desktop/)
- 如需图形界面支持，安装 [XQuartz](https://www.xquartz.org/) 并启用"允许来自网络客户端的连接"选项
- USB连接的Android设备（用于scrcpy）

## 使用方法

1. 克隆此仓库到您的本地机器
2. 使用以下命令启动 Docker 开发环境：

```bash
# 赋予脚本执行权限
chmod +x build.sh start_ros_system.sh stop_ros_system.sh rebuild.sh fix_ultralytics.sh run_without_build.sh

# 启动 ROS Docker 开发环境
./start_ros_system.sh
```

3. 在容器内，您可以使用标准 ROS 命令进行开发和测试：

```bash
# 确保虚拟环境已激活
source /opt/venv/bin/activate

# 构建工作空间
colcon build

# 加载环境
source install/setup.bash

# 运行检测节点
ros2 run scrcpy_ros scrcpy_publisher
```

## 关于Python虚拟环境

本项目使用Python虚拟环境来解决依赖问题。在容器内，虚拟环境位于`/opt/venv`目录。开启容器时会自动激活虚拟环境，如果需要手动激活，可以运行：

```bash
source /opt/venv/bin/activate
```

所有Python包(包括ultralytics)都安装在虚拟环境中，不会与系统Python冲突。

## 故障排除

如果遇到 ultralytics 模块导入错误，在容器内运行以下命令修复：

```bash
./fix_ultralytics.sh
```

这个脚本会：
- 确保虚拟环境正确激活
- 更新 pip 并安装必要的依赖
- 尝试重新安装 ultralytics 和 PyTorch
- 配置 Python 路径

### 常见问题及解决方案

1. **ModuleNotFoundError: No module named 'ultralytics'**
   
   确保已激活虚拟环境，然后运行修复脚本：
   ```bash
   source /opt/venv/bin/activate
   ./fix_ultralytics.sh
   ```

2. **externally-managed-environment错误**
   
   这是由于系统Python保护机制（PEP 668）导致的。解决方法是确保在虚拟环境中安装包，而不是系统Python环境。

3. **找不到共享库或依赖项**
   
   检查PYTHONPATH环境变量是否正确设置：
   ```bash
   echo $PYTHONPATH
   ```
   
   如有必要，手动设置：
   ```bash
   export PYTHONPATH=$PYTHONPATH:/opt/venv/lib/python3.12/site-packages:/ros_ws/install/lib/python3.12/site-packages
   ```

## 不重新构建的情况下启动容器

如果您只想启动现有容器而不重新构建：

```bash
./run_without_build.sh
```

## 重建容器

如果您需要完全重建容器（例如，当您对 Dockerfile 进行更改时）：

```bash
./rebuild.sh
```

## 停止环境

当您完成工作后，可以使用以下命令停止并移除容器：

```bash
./stop_ros_system.sh
```

## 项目说明

此项目已从使用Darknet YOLOv3/v4升级到使用YOLOv8，提供了更高的检测精度和性能。环境配置了必要的Python虚拟环境和依赖项，确保YOLOv8可以正常运行，同时避免与系统Python冲突。

## 项目结构

- `src/` - ROS 源代码目录，与主机共享
- `Dockerfile` - Docker 镜像定义（已集成YOLOv8支持和虚拟环境）
- `docker-compose.yml` - Docker 容器配置
- `build.sh` - 用于构建 Docker 镜像的脚本
- `start_ros_system.sh` - 用于启动开发环境的脚本
- `stop_ros_system.sh` - 用于停止开发环境的脚本
- `rebuild.sh` - 用于从头重建Docker镜像的脚本
- `fix_ultralytics.sh` - 容器内修复ultralytics安装的脚本
- `run_without_build.sh` - 不重新构建直接运行容器的脚本
