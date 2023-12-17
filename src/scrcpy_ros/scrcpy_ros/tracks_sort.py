import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from lane_msg.msg import Detection
import cv2
import numpy as np

class Tracker:
    def __init__(self, tracker_id, bbox):
        self.id = tracker_id
        self.bbox = bbox  # 边界框格式: [x, y, w, h]
        self.lost = 0     # 跟踪丢失的帧数

    def update(self, new_bbox):
        self.bbox = new_bbox
        self.lost = 0  # 成功更新后重置丢失计数

    def increment_lost(self):
        self.lost += 1


class TracksSort(Node):
    def __init__(self):
        super().__init__('tracks_sort')

        self.bridge = CvBridge()
        self.current_detections = []

        self.trackers = {}  # 存储目标和对应的跟踪器
        self.next_id = 0    # 用于生成下一个唯一 ID

        self.trackers_index = {}

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

        updated_trackers = {}

        width_ratio = float(new_width) / 416
        height_ratio = float(new_height) / 416

        frame_shape = (new_height, new_width, 3)
        frame = cv_image_resized

        class_ids = []
        confidences = []
        boxes = []
        for detection in self.current_detections:
            class_id = detection.class_name
            confidence = detection.confidence
            if confidence > 0.5:  # 设置置信度阈值
                x = int(detection.x * width_ratio)  # 将 x 坐标调整到缩放后的图像尺寸
                y = int(detection.y * height_ratio) # 将 y 坐标调整到缩放后的图像尺寸
                w = int(detection.width * width_ratio)  # 将宽度调整到缩放后的图像尺寸
                h = int(detection.height * height_ratio) # 将高度调整到缩放后的图像尺寸

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

        # 应用非极大值抑制
        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        detections = []
        for i in indices:
            box = boxes[i]
            detections.append([box[0], box[1], box[0]+box[2], box[1]+box[3], confidences[i]])


        for i, det in enumerate(detections):
            matched, tracker_id = self.match_detection_with_trackers(det)
            if matched:
                self.trackers[tracker_id].update(det)
                updated_trackers[tracker_id] = self.trackers[tracker_id]
                self.trackers_index[tracker_id] = i  # 更新跟踪器索引
            else:
                new_tracker_id = self.next_id
                self.next_id += 1
                new_tracker = Tracker(new_tracker_id, det)
                updated_trackers[new_tracker_id] = new_tracker
                self.trackers_index[new_tracker_id] = i  # 添加新跟踪器索引


        # 更新跟踪器列表
        self.trackers = updated_trackers
        
        for tracker_id, i in self.trackers_index.items():
            # 提取跟踪器的边界框信息
            detection = self.current_detections[i]
            x, y, w, h = int(detection.x * width_ratio), int(detection.y * height_ratio), int(detection.width * width_ratio), int(detection.height * height_ratio)
            left = int(x - w / 2)
            top = int(y - h / 2)
            right = int(x + w / 2)
            bottom = int(y + h / 2)

            cv2.rectangle(cv_image_resized, (left, top), (right, bottom), (0, 255, 0), 2)
            cv2.putText(cv_image_resized, detection.class_name, (left, top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            self.get_logger().info(f"Drawing detection: {detection.class_name} ({detection.confidence})")      

        """
        
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
        """

        self.current_detections.clear()
        self.trackers_index.clear()
        
        cv2.imshow('Tracking', cv_image_resized)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()


    def match_detection_with_trackers(self, detection):
        max_iou = 0
        best_tracker_id = None
        for tracker_id, tracker in self.trackers.items():
            iou = self.calculate_iou(tracker.bbox, detection)
            if iou > max_iou:
                max_iou = iou
                best_tracker_id = tracker_id

        iou_threshold = 0.3  # 可调整的 IoU 阈值
        if max_iou > iou_threshold:
            return True, best_tracker_id
        else:
            return False, None

    def calculate_iou(self, box1, box2):
        # 计算边界框的 (x, y) 坐标
        x1_min, y1_min, x1_max, y1_max = box1[0], box1[1], box1[0] + box1[2], box1[1] + box1[3]
        x2_min, y2_min, x2_max, y2_max = box2[0], box2[1], box2[0] + box2[2], box2[1] + box2[3]

        # 计算两个边界框的交集区域
        intersect_x_min = max(x1_min, x2_min)
        intersect_y_min = max(y1_min, y2_min)
        intersect_x_max = min(x1_max, x2_max)
        intersect_y_max = min(y1_max, y2_max)

        # 计算交集区域的面积
        intersect_area = max(intersect_x_max - intersect_x_min, 0) * max(intersect_y_max - intersect_y_min, 0)

        # 计算每个边界框的面积
        box1_area = (x1_max - x1_min) * (y1_max - y1_min)
        box2_area = (x2_max - x2_min) * (y2_max - y2_min)

        # 计算并集区域的面积
        union_area = box1_area + box2_area - intersect_area

        # 计算 IoU
        iou = intersect_area / union_area if union_area != 0 else 0

        return iou

def main(args=None):
    rclpy.init(args=args)

    tracks_sort = TracksSort()

    rclpy.spin(tracks_sort)

    # 销毁节点并清理
    tracks_sort.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()