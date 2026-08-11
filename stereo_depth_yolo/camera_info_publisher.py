#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml
import os

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        
        self.declare_parameter('left_yaml', '')
        self.declare_parameter('right_yaml', '')
        
        left_yaml_path = self.get_parameter('left_yaml').value
        right_yaml_path = self.get_parameter('right_yaml').value
        
        self.left_info = self.load_yaml(left_yaml_path)
        self.right_info = self.load_yaml(right_yaml_path)
        
        self.left_pub = self.create_publisher(CameraInfo, '/left/camera_info', 10)
        self.right_pub = self.create_publisher(CameraInfo, '/right/camera_info', 10)
        
        self.sub = self.create_subscription(CameraInfo, '/left/camera_info_raw', self.callback, 10)
        self.get_logger().info(
            f"CameraInfo Publisher Initialized ({self.left_info.width}x{self.left_info.height} from calibration)."
        )

    def load_yaml(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f"YAML file not found: {path}")
            return CameraInfo()
            
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        
        msg = CameraInfo()
        # Automatically pull dimensions from calibration file
        msg.width = int(data.get('image_width', 1280))
        msg.height = int(data.get('image_height', 720))
        msg.distortion_model = data.get('distortion_model', 'plumb_bob')
        msg.d = [float(x) for x in data['distortion_coefficients']['data']]
        msg.k = [float(x) for x in data['camera_matrix']['data']]
        msg.r = [float(x) for x in data['rectification_matrix']['data']]
        msg.p = [float(x) for x in data['projection_matrix']['data']]
        return msg

    def callback(self, msg):
        self.left_info.header = msg.header
        self.left_info.header.frame_id = 'left_camera_link'
        
        self.right_info.header = msg.header
        self.right_info.header.frame_id = 'right_camera_link'
        
        self.left_pub.publish(self.left_info)
        self.right_pub.publish(self.right_info)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
