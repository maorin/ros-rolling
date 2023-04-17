import rclpy
from rclpy.node import Node



from lane_msg.msg import Detection

class DetectionSubscriber(Node):
    def __init__(self):
        super().__init__('detection_subscriber')

        self.subscription = self.create_subscription(
            Detection,
            'detections',
            self.detection_callback,
            10
        )

    def detection_callback(self, msg):
        print(msg.class_name)
        self.get_logger().info('Received detection: %s (%f)' % (msg.class_name, msg.confidence))

def main(args=None):
    rclpy.init(args=args)

    detection_subscriber = DetectionSubscriber()

    rclpy.spin(detection_subscriber)

    # 销毁节点并清理
    detection_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()