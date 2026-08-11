import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml
import os
import threading
import time

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

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # Explicitly control exposure & brightness
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3) # V4L2 Auto Exposure
        # Optional manual overrides if auto-exposure stays dark:
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # Manual Mode
        # self.cap.set(cv2.CAP_PROP_EXPOSURE, 300)
        # self.cap.set(cv2.CAP_PROP_GAIN, 30)
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
            
        # Target height is 736 to account for 16px bottom padding
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
                time.sleep(0.01)
                continue

            half_w = frame.shape[1] // 2
            left_crop = frame[:, :half_w]
            right_crop = frame[:, half_w:]

            # Add 16px bottom padding to reach 1280x736 (Zero CPU interpolation overhead)
            left_eye = cv2.copyMakeBorder(left_crop, 0, 16, 0, 0, cv2.BORDER_CONSTANT, value=[114, 114, 114])
            right_eye = cv2.copyMakeBorder(right_crop, 0, 16, 0, 0, cv2.BORDER_CONSTANT, value=[114, 114, 114])

            left_msg = self.bridge.cv2_to_imgmsg(left_eye, "bgr8")
            right_msg = self.bridge.cv2_to_imgmsg(right_eye, "bgr8")

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
