import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from lane_msg.msg import Detection
import cv2
import numpy as np

class DetectionSubscriber(Node):
    def __init__(self):
        super().__init__('detection_subscriber')

        self.bridge = CvBridge()
        self.current_detections = []
        x, y, w, h = 100, 100, 50, 50  # 假设的初始边界框
        empty_image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        self.subscription = self.create_subscription(
            Detection,
            'detections',
            self.detection_callback,
            10
        )

        self.video_subscription = self.create_subscription(
            Image,
            'scrcpy_image',
            self.video_callback,
            10
        )


    def detection_callback(self, msg):
        self.current_detections.append(msg)
        self.get_logger().info('Received detection: %s (%f)' % (msg.class_name, msg.confidence))

    def resize_image_with_aspect_ratio(self, image, new_width):
        height, width, _ = image.shape
        aspect_ratio = float(width) / float(height)
        new_height = int(new_width / aspect_ratio)
        image_resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
        return image_resized, new_width, new_height


    def calculate_distance(self, detection, frame_center):
        x, y, w, h = detection.x, detection.y, detection.width, detection.height
        bbox_center = (x + w / 2, y + h / 2)
        return np.linalg.norm(np.array(frame_center) - np.array(bbox_center))

    def sort_detections(self, detections, frame_shape):
        frame_center = (frame_shape[1] / 2, frame_shape[0] / 2)
        sorted_detections = sorted(detections, key=lambda det: (self.calculate_distance(det, frame_center), -det.width * det.height))
        return sorted_detections

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

        sorted_detections = self.sort_detections(self.current_detections, frame_shape)



        if not sorted_detections:
            return



        # 处理排序后的检测结果
        for detection in sorted_detections:
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

    detection_subscriber = DetectionSubscriber()

    rclpy.spin(detection_subscriber)

    # 销毁节点并清理
    detection_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()