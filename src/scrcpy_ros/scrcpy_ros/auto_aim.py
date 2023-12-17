import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from lane_msg.msg import Detection
import cv2
import numpy as np

class AutoAim(Node):
    def __init__(self):
        super().__init__('auto_aim')

        self.bridge = CvBridge()
        self.current_detections = []
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


    def largest_target_detection_callback(self, msg):
        self.current_detections.append(msg)
        self.get_logger().info('Received largest_target: %s (%f)' % (msg.class_name, msg.confidence))
        #在这里添加自瞄代码




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


        # 处理排序后的检测结果
        for detection in self.current_detections:
            # Draw the detection on the frame
            x, y, w, h = int(detection.x * width_ratio), int(detection.y * height_ratio), int(detection.width * width_ratio), int(detection.height * height_ratio)
            left = int(x - w / 2)
            top = int(y - h / 2)
            right = int(x + w / 2)
            bottom = int(y + h / 2)

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