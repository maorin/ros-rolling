from ppadb.client import Client as AdbClient

def swipe(device, x1, y1, x2, y2, duration):
    device.shell(f"input swipe {x1} {y1} {x2} {y2} {duration}")

# 初始化ADB客户端
client = AdbClient(host="127.0.0.1", port=5037)

# 获取已连接的设备
devices = client.devices()

if len(devices) == 0:
    print("未找到已连接的设备")
else:
    # 使用第一个已连接的设备
    device = devices[0]

    # 模拟滑动操作，从坐标 (100, 100) 到 (200, 200)，持续时间为 500 毫秒
    swipe(device, 100, 100, 800, 800, 500)
