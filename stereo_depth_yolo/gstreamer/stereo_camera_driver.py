#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml
import os
import threading
import time
import numpy as np

class StereoCameraDriver(Node):
    def __init__(self):
        super().__init__('stereo_camera_driver')
        
        self.left_pub = self.create_publisher(Image, '/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, '/right/image_raw', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, '/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/right/camera_info', 10)
        
        self.bridge = CvBridge()
        self.target_width = 1280
        self.target_height = 736

        left_yaml = '/workspaces/isaac_ros-dev/src/stereo_depth_yolo/config/left.yaml'
        right_yaml = '/workspaces/isaac_ros-dev/src/stereo_depth_yolo/config/right.yaml'
        
        self.left_info_msg = self.load_camera_info(left_yaml)
        self.right_info_msg = self.load_camera_info(right_yaml)

        # High-Speed Option A GStreamer Pipeline
        gst_pipeline = (
            "v4l2src device=/dev/video0 io-mode=2 ! "
            "image/jpeg, width=2560, height=720, framerate=30/1 ! "
            "jpegdec ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )

        self.get_logger().info("Initializing High-Speed GStreamer Pipeline on /dev/video0...")
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        # Fallback to standard V4L2 if GStreamer pipeline fails
        if not self.cap.isOpened():
            self.get_logger().warn("GStreamer failed to open device. Falling back to standard V4L2...")
            self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            self.cap.set(cv2.CAP_PROP_FPS, 60)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)


        self.left_buffer = np.full((736, 1280, 3), [114, 114, 114], dtype=np.uint8)
        self.right_buffer = np.full((736, 1280, 3), [114, 114, 114], dtype=np.uint8)
        self.running = True
        self.thread = threading.Thread(target=self.poll_camera)
        self.thread.daemon = True
        self.thread.start()

    def load_camera_info(self, yaml_path):
        msg = CameraInfo()
        if not os.path.exists(yaml_path):
            self.get_logger().warn(f"Cannot find calibration file {yaml_path}")
            return msg
            
        with open(yaml_path, 'r') as file:
            calib = yaml.safe_load(file)
            
        msg.width = 1280
        msg.height = 736
        msg.distortion_model = calib.get('distortion_model', 'plumb_bob')
        msg.d = list(calib.get('distortion_coefficients', {}).get('data', []))
        msg.k = list(calib.get('camera_matrix', {}).get('data', [0.0]*9))
        msg.r = list(calib.get('rectification_matrix', {}).get('data', [0.0]*9))
        msg.p = list(calib.get('projection_matrix', {}).get('data', [0.0]*12))

        return msg

    def poll_camera(self):
        while self.running and rclpy.ok():
            ret, frame = self.cap.read()
            if not ret or frame is None:
                time.sleep(0.005)
                continue

            # Direct NumPy memory block assignment (Zero memory allocation)
            self.left_buffer[:720, :] = frame[:, :1280]
            self.right_buffer[:720, :] = frame[:, 1280:]

            # Convert to ROS messages directly from the pre-allocated buffers
            left_msg = self.bridge.cv2_to_imgmsg(self.left_buffer, "bgr8")
            right_msg = self.bridge.cv2_to_imgmsg(self.right_buffer, "bgr8")

            now = self.get_clock().now().to_msg()
            left_msg.header.stamp = now
            left_msg.header.frame_id = 'camera_link'
            self.left_info_msg.header = left_msg.header

            right_msg.header.stamp = now
            right_msg.header.frame_id = 'camera_link'
            self.right_info_msg.header = right_msg.header

            self.left_pub.publish(left_msg)
            self.right_pub.publish(right_msg)
            self.left_info_pub.publish(self.left_info_msg)
            self.right_info_pub.publish(self.right_info_msg)

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    driver = StereoCameraDriver()
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
