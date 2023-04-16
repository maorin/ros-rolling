import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import subprocess
import os
import sys
import scrcpy_ros.darknet as dn


class ScrcpyPublisher(Node):
    def __init__(self):
        super().__init__('scrcpy_publisher')
        self.publisher_ = self.create_publisher(Image, 'scrcpy_image', 10)
        self.bridge = CvBridge()

        # Set up YOLOv3 model
        self.yolo_net = cv2.dnn.readNet("/data/yolo/yolov3.weights", "/data/yolo/yolov3.cfg")
        self.yolo_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.yolo_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    
        # Load object classes
        with open("/data/yolo/coco.names", "rt") as f:
            self.classes = f.read().rstrip('\n').split('\n')

    def detect_objects(self, frame):
        # Preprocess the frame
        blob = cv2.dnn.blobFromImage(frame, 1 / 255, (416, 416), (0, 0, 0), True, crop=False)
        self.yolo_net.setInput(blob)
    
        # Get output layers
        layer_names = self.yolo_net.getLayerNames()
        output_layer_indexes = self.yolo_net.getUnconnectedOutLayers().flatten()
        output_layers = [layer_names[i - 1] for i in output_layer_indexes]
    
        # Forward pass and get detections
        detections = self.yolo_net.forward(output_layers)
    
        return detections



    def create_v4l2_device(self):
        # Check if v4l2loopback module is loaded
        module_loaded = subprocess.run(["lsmod"], capture_output=True, text=True)
        if "v4l2loopback" not in module_loaded.stdout:
            print("v4l2loopback kernel module not loaded. Loading now...")
            subprocess.run(["sudo", "modprobe", "v4l2loopback"])
    
        # Check if a v4l2loopback device already exists
        list_devices = subprocess.run(["v4l2-ctl", "--list-devices"], capture_output=True, text=True)
        video_device = None
        for line in list_devices.stdout.splitlines():
            if "Dummy video device" in line:
                video_device = subprocess.run(["v4l2-ctl", "--list-ctrls", "--device", line.split("(")[-1].strip(")")], capture_output=True, text=True)
                for sub_line in video_device.stdout.splitlines():
                    if "index" in sub_line:
                        index = int(sub_line.split("=")[-1])
                        video_device = f"/dev/video{index}"
                        break
                break
    
        # If no v4l2loopback device is found, create one
        if not video_device:
            print("No v4l2loopback device found. Creating one now...")
    
            # Find the first available video device
            for i in range(10):
                device = f"/dev/video{i}"
                if os.path.exists(device):
                    try:
                        cap = cv2.VideoCapture(device)
                        if not cap.isOpened():
                            video_device = device
                            break
                    finally:
                        cap.release()
    
        if not video_device:
            print("No available video device found.")
            sys.exit(1)
    
        return video_device





    def capture_and_publish(self):
        
        video_device = self.create_v4l2_device()
        print("-------------%s" % video_device)
    
        # Start scrcpy in the background
        scrcpy_process = subprocess.Popen(["scrcpy", f"--no-display", f"--v4l2-sink=/dev/video0"])
        


        # Open virtual video device
        cap = cv2.VideoCapture('/dev/video0')

        # 设置 Darknet 网络参数
        config_file = "/data/yolo/yolov3.cfg"
        data_file = "/home/maojj/project/darknet/cfg/coco.data"
        weights_file = "/data/yolo/yolov3.weights"

        # 加载 Darknet 网络
        network, class_names, class_colors = dn.load_network(config_file, data_file, weights_file)

        # 获取网络输入的宽度和高度
        darknet_width = dn.network_width(network)
        darknet_height = dn.network_height(network)

        while rclpy.ok():
            # Capture frame from video device
            ret, frame = cap.read()

            if ret:
                # 将帧从 BGR 转换为 RGB，然后调整尺寸以适应 Darknet 网络
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame_resized = cv2.resize(frame_rgb, (darknet_width, darknet_height), interpolation=cv2.INTER_LINEAR)
                
                # 将 frame_resized 转换为 IMAGE 类型
                frame_resized_image = dn.make_image(frame_resized.shape[1], frame_resized.shape[0], frame_resized.shape[2])
                dn.copy_image_from_bytes(frame_resized_image, frame_resized.tobytes())
                
                # 使用 Darknet 网络对帧进行处理
                detections = dn.detect_image(network, class_names, frame_resized_image)
                print("------------11111111-----------------")
                print(detections)
                print("------------2222222222-------------------------")

                # Convert OpenCV image to ROS 2 image message
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.publisher_.publish(ros_image)

            else:
                print("Error: Unable to read frame.")
                break



        # Close OpenCV window
        cv2.destroyAllWindows()
        cap.release()

        # Stop scrcpy process
        scrcpy_process.terminate()

def main(args=None):
    rclpy.init(args=args)
    scrcpy_publisher = ScrcpyPublisher()
    scrcpy_publisher.capture_and_publish()
    scrcpy_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
