import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from lane_msg.msg import Detection
import cv2
import numpy as np
from ppadb.client import Client as AdbClient
import threading

class AutoAim(Node):
    def __init__(self):
        super().__init__('auto_aim')

        self.bridge = CvBridge()
        self.current_detections = []
        self.target_position = None
        self.adb_commands = []
        self.client = AdbClient(host="127.0.0.1", port=5037)
        self.devices = self.client.devices()

        # 初始化锁
        self.lock = threading.Lock()
        self.command_thread = None

        x, y, w, h = 100, 100, 50, 50  # 假设的初始边界框
        empty_image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        self.subscription = self.create_subscription(
            Detection,
            'largest_target',
            self.largest_target_detection_callback,
            10
        )

        self.video_subscription = self.create_subscription(
            Image,
            'scrcpy_image',
            self.video_callback,
            10
        )

    def execute_adb_command(self, lock, device, command):
        with lock:

            print("runing  command:",command)
            device.shell(f"input swipe {command[0]} {command[1]} {command[2]} {command[3]} {command[4]}")
            # 命令执行完成后，锁会自动释放

    def swipe(self, device, x1, y1, x2, y2, duration):
        device.shell(f"input swipe {x1} {y1} {x2} {y2} {duration}")

    def generate_adb_commands(self, start_x, start_y, displacement_x, displacement_y, steps):
        commands = []
        step_x = displacement_x // steps
        step_y = displacement_y // steps
        current_x, current_y = start_x, start_y

        for _ in range(steps):
            next_x = current_x + step_x
            next_y = current_y + step_y
            commands.append((current_x, current_y, next_x, next_y, 200))
            current_x, current_y = next_x, next_y

        return commands

    def calculate_displacement(self, screen_center_x, screen_center_y, target_x, target_y):
        displacement_x = target_x - screen_center_x
        displacement_y = target_y - screen_center_y
        return displacement_x, displacement_y

    def is_point_inside_rectangle(self, left, top, right, bottom, center_x, center_y):
        if left <= center_x <= right and top <= center_y <= bottom:
            return True
        else:
            return False
    

    def generate_adb_swipe_command(self, start_x, start_y, end_x, end_y, swipe_percentage):
        """
        生成ADB滑动命令。

        :param start_x: 滑动起始点的x坐标。
        :param start_y: 滑动起始点的y坐标。
        :param end_x: 滑动终点的x坐标。
        :param end_y: 滑动终点的y坐标。
        :param swipe_percentage: 滑动距离占线段总长度的百分比。
        :return: ADB滑动命令的字符串。
        """
        duration = 200  # 滑动持续时间
        # 计算方向向量
        direction_x = end_x - start_x
        direction_y = end_y - start_y

        # 计算百分比处的点
        length_percentage_x = direction_x * swipe_percentage
        length_percentage_y = direction_y * swipe_percentage

        # 计算百分比处的坐标
        percentage_point_x = start_x + length_percentage_x
        percentage_point_y = start_y + length_percentage_y

        # 生成ADB滑动命令
        adb_start_x = int(start_x)
        adb_start_y = int(start_y)
        adb_end_x = int(percentage_point_x)
        adb_end_y = int(percentage_point_y)

        # 原始分辨率和目标分辨率
        original_resolution_x, original_resolution_y = 800, 416
        target_resolution_x, target_resolution_y = 2340, 1080

        # 计算宽度和高度的比例
        x_ratio = target_resolution_x / original_resolution_x
        y_ratio = target_resolution_y / original_resolution_y

        # 应用比例到原始坐标
        adb_start_x = int(adb_start_x * x_ratio)
        adb_start_y = int(adb_start_y * y_ratio)
        adb_end_x = int(adb_end_x * x_ratio)
        adb_end_y = int(adb_end_y * y_ratio)

        return [(adb_start_x, adb_start_y, adb_end_x, adb_end_y, duration)]




    def largest_target_detection_callback(self, msg):
        self.current_detections = [msg]
        self.get_logger().info('Received largest_target: %s (%f)' % (msg.class_name, msg.confidence))
    
        if self.command_thread:
            self.command_thread.join()

        for command in self.adb_commands:
            if self.devices:
                device = self.devices[0]

                self.command_thread = threading.Thread(
                    target=self.execute_adb_command, 
                    args=(self.lock, device, command))
                self.command_thread.start()



    def resize_image_with_aspect_ratio(self, image, new_width):
        height, width, _ = image.shape
        aspect_ratio = float(width) / float(height)
        new_height = int(new_width / aspect_ratio)
        image_resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
        return image_resized, new_width, new_height


    def video_callback(self, img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')

            cv_image_resized, new_width, new_height = self.resize_image_with_aspect_ratio(cv_image, 800)
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

        width_ratio = float(new_width) / 416
        height_ratio = float(new_height) / 416

        frame_shape = (new_height, new_width, 3)
        

        #在这里检测目标是不是在视野中
        if self.current_detections:
            detection = self.current_detections[0]

            x, y, w, h = int(detection.x), int(detection.y), int(detection.width), int(detection.height)
            left = int(x - w / 2)
            top = int(y - h / 2)
            right = int(x + w / 2)
            bottom = int(y + h / 2)
            center_x = left + (w / 2)
            center_y = top + (h / 2)

            print("center_x:",center_x)
            print("center_y:",center_y)


            # 缩小后的屏幕中心点
            small_screen_center_x = 800 / 2
            small_screen_center_y = 416 / 2
            

            # 判断准心在不在目标内
            if not self.is_point_inside_rectangle(left, top, right, bottom, small_screen_center_x, small_screen_center_y):
                self.get_logger().info('Target is inside the screen')
                
                # 目标在屏幕中的坐标
                target_x = center_x * 2340 / 800
                target_y = center_y * 1080 / 416

                # 屏幕中心点
                screen_center_x = 2340 / 2
                screen_center_y = 1080 / 2


                # 按住屏幕的初始坐标
                hold_init_x =  (2340 / 2) + 500
                hold_init_y =  1080 / 2

                small_hold_init_x = hold_init_x * (800 / 2340)
                small_hold_init_y = hold_init_y * (416 / 1080)

                
                print("hold_init_x:",hold_init_x)
                print("hold_init_y:",hold_init_y)

                # 生成滑动命令
                #displacement_x, displacement_y = self.calculate_displacement(small_screen_center_x, small_screen_center_y, center_x, center_y)

                #self.adb_commands = self.generate_adb_commands(screen_center_x, screen_center_y, displacement_x, displacement_y, steps=1)

                # 从屏幕中心到目标点的线    
                # 将浮点坐标转换为整数
                small_screen_center_x = int(small_screen_center_x)
                small_screen_center_y = int(small_screen_center_y)
                center_x = int(center_x)
                center_y = int(center_y)  
                print("small_screen_center_x:",small_screen_center_x)
                print("small_screen_center_y:",small_screen_center_y)
                print("center_x:",center_x)
                print("center_y:",center_y)
                            
                cv2.line(cv_image_resized, (small_screen_center_x, small_screen_center_y), (center_x, center_y), (255, 0, 0), 2)
                
                x_ratio =  800 / 2340
                y_ratio =  416 / 1080


                # 原始线段的起点和终点
                original_start_x, original_start_y = small_screen_center_x, small_screen_center_y
                original_end_x, original_end_y = center_x, center_y

                # 计算原始线段的方向向量
                direction_x = original_end_x - original_start_x
                direction_y = original_end_y - original_start_y

                # 新线段的起点
                new_start_x = int(small_hold_init_x)
                new_start_y = int(small_hold_init_y)

                # 计算新线段的终点
                new_end_x = int(new_start_x + direction_x)
                new_end_y = int(new_start_y + direction_y)

                # 画出新线段
                cv2.line(cv_image_resized, (new_start_x, new_start_y), (new_end_x, new_end_y), (255, 0, 0), 2)

                swipe_percentage = 0.1 # 滑动距离占线段总长度的百分比

                adb_command = self.generate_adb_swipe_command(new_start_x, new_start_y, new_end_x, new_end_y, swipe_percentage)

                #self.adb_commands.append(adb_command)
                self.adb_commands = adb_command


                """
                # 画出ADB命令模拟的滑动线
                for command in self.adb_commands:
                    start_x, start_y, end_x, end_y, _ = command
                    cv2.line(cv_image_resized, (int(start_x * x_ratio), int(start_y * y_ratio)), (int(end_x * x_ratio), int(end_y * y_ratio)), (33, 24, 255), 2)
                """
            else:
                self.get_logger().info('Target is outside the screen')
                self.adb_commands = []  # 清空命令


            # 从屏幕中心到目标点的线
            cv2.rectangle(cv_image_resized, (left, top), (right, bottom), (0, 255, 0), 2)
            cv2.putText(cv_image_resized, detection.class_name, (left, top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            self.get_logger().info(f"Drawing detection: {detection.class_name} ({detection.confidence})")


            self.current_detections.clear()
            
            cv2.imshow('Video Stream with Detections', cv_image_resized)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    detection_autoaim = AutoAim()

    rclpy.spin(detection_autoaim)

    # 销毁节点并清理
    detection_autoaim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()