import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import subprocess
import os
import sys

class ScrcpyPublisher(Node):
    def __init__(self):
        super().__init__('scrcpy_publisher')
        self.publisher_ = self.create_publisher(Image, 'scrcpy_image', 10)
        self.bridge = CvBridge()

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
        scrcpy_process = subprocess.Popen(["scrcpy", f"--v4l2-sink=/dev/video0"])
        


        # Open virtual video device
        cap = cv2.VideoCapture('/dev/video0')

        # Create an OpenCV window
        cv2.namedWindow("Scrcpy Stream", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Scrcpy Stream", 800, 480)

        while rclpy.ok():
            # Capture frame from video device
            ret, frame_data = cap.read()

            if ret:
                # Display frame in OpenCV window
                cv2.imshow("Scrcpy Stream", frame_data)
                cv2.waitKey(1)

                # Convert OpenCV image to ROS 2 image message
                ros_image = self.bridge.cv2_to_imgmsg(frame_data, "bgr8")
                self.publisher_.publish(ros_image)

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
