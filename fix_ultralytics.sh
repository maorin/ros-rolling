#!/bin/bash

# 该脚本应在Docker容器内部运行，用于修复ultralytics库的安装问题

echo "修复ultralytics安装..."

# 确保虚拟环境已激活
if [ -d "/opt/venv" ]; then
  echo "检测到虚拟环境，激活中..."
  source /opt/venv/bin/activate
else
  echo "警告：未检测到虚拟环境，将创建一个新的..."
  python3 -m venv /opt/venv
  source /opt/venv/bin/activate
fi

# 更新pip和基础依赖
pip install --upgrade pip
pip install setuptools wheel

# 尝试更好地安装ultralytics
echo "重新安装ultralytics..."
pip uninstall -y ultralytics || true
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
pip install -U ultralytics

# 测试导入ultralytics
echo "测试导入ultralytics..."
python -c "
try:
    from ultralytics import YOLO
    print('成功导入ultralytics库!')
    
    # 尝试加载模型
    try:
        model = YOLO('yolov8n.pt')
        print('成功加载YOLOv8模型!')
    except Exception as e:
        print(f'加载模型时出错: {e}')
except ImportError as e:
    print(f'导入ultralytics库时出错: {e}')
"

# 配置Python路径
echo "配置Python路径..."
VENV_SITE_PACKAGES=$(python -c "import site; print(site.getsitepackages()[0])")
export PYTHONPATH=$PYTHONPATH:$VENV_SITE_PACKAGES:/ros_ws/install/lib/python3.12/site-packages

# 更新bashrc
echo "更新环境变量..."
grep -q "source /opt/venv/bin/activate" ~/.bashrc || echo "source /opt/venv/bin/activate" >> ~/.bashrc
grep -q "PYTHONPATH" ~/.bashrc || echo "export PYTHONPATH=\$PYTHONPATH:$VENV_SITE_PACKAGES:/ros_ws/install/lib/python3.12/site-packages" >> ~/.bashrc

echo "完成修复，请尝试运行: ros2 run scrcpy_ros scrcpy_publisher"
echo "如果遇到问题，请确保已激活虚拟环境: source /opt/venv/bin/activate" 