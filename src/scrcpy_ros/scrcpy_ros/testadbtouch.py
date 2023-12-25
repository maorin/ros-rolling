import re
import subprocess

"""
[  119309.943905] EV_SYN       SYN_REPORT           00000000            
[  119309.953220] EV_ABS       ABS_MT_POSITION_X    0000028b            
[  119309.953220] EV_ABS       ABS_MT_POSITION_Y    0000054f            
[  119309.953220] EV_ABS       ABS_MT_PRESSURE      0000016d            
[  119309.953220] EV_ABS       ABS_MT_TRACKING_ID   00000000            
[  119309.953220] EV_ABS       ABS_MT_TOUCH_MAJOR   000000af            
[  119309.953220] EV_ABS       ABS_MT_TOUCH_MINOR   0000009f            
[  119309.953220] EV_ABS       ABS_MT_ORIENTATION   ffffffbe            
[  119309.953220] EV_ABS       ABS_MT_BLOB_ID       00000002            
[  119309.953220] EV_SYN       SYN_MT_REPORT        00000000            
[  119309.953220] EV_SYN       SYN_REPORT           00000000            
[  119309.962042] EV_ABS       ABS_MT_POSITION_X    0000028b            
[  119309.962042] EV_ABS       ABS_MT_POSITION_Y    00000550            
[  119309.962042] EV_ABS       ABS_MT_PRESSURE      000000d2            
[  119309.962042] EV_ABS       ABS_MT_TRACKING_ID   00000000            
[  119309.962042] EV_ABS       ABS_MT_TOUCH_MAJOR   000000bf            
[  119309.962042] EV_ABS       ABS_MT_TOUCH_MINOR   000000af            
[  119309.962042] EV_ABS       ABS_MT_ORIENTATION   ffffffaa        

### 使用`sendevent`模拟压力（高级用法）

1. **查找触摸屏设备路径**:
   使用`getevent -lp`命令找到触摸屏设备的路径。

2. **发送低级触摸事件**:
   使用`sendevent`命令发送特定的事件代码和值，包括压力级别。

例如：
```bash
adb shell sendevent /dev/input/eventX <type> <code> <value>
```

- `/dev/input/eventX`是触摸屏设备的路径。
- `<type>`, `<code>`, `<value>`是特定于设备的事件类型、事件代码和值，需要根据具体设备进行调整。

这种方法需要对Android的输入子系统有深入了解，并且可能需要根据不同的设备和驱动程序进行调整。因此，一般仅在开发或测试环境中由经验丰富的开发人员使用。

对于常规应用和测试，通常不需要模拟压力，因为大多数应用程序和用户界面交互并不依赖于压力数据。如果你确实需要进行这样的模拟，可能需要考虑使用专门的测试工具或开发自定义解决方案。


"""



def listen_touch_events():
    # 正则表达式匹配触摸坐标
    #pattern = re.compile(r"ABS_MT_POSITION_(X|Y)\s+\w+\s+(\w+)")
    pattern = re.compile(r"ABS_MT_POSITION_(X|Y)\s+.*\s+(\w+)")


    # 启动ADB命令获取触摸事件
    adb_command = ["adb", "shell", "getevent", "-lt", "/dev/input/event4"]
    process = subprocess.Popen(adb_command, stdout=subprocess.PIPE, text=True)

    x, y = None, None

    try:
        while True:
            line = process.stdout.readline()
            if not line:
                break

            match = pattern.search(line)
            if match:
                print(match.groups())
                axis, position_hex = match.groups()
                position = int(position_hex, 16)  # 转换为十进制

                if axis == 'X':
                    x = position
                elif axis == 'Y':
                    y = position
                    # 当匹配到Y坐标时，如果之前已匹配到X坐标，则一起打印
                    if x is not None:
                        print("Touch Position:", (x, y))
                        x, y = None, None  # 重置坐标
    finally:
        process.kill()

listen_touch_events()
