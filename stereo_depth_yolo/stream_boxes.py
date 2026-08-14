#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2

class HeadlessVisualizer(Node):
    def __init__(self):
        super().__init__('headless_visualizer')
        self.bridge = CvBridge()
        
        # CRITICAL: Use qos_profile_sensor_data to match Isaac ROS Best-Effort QoS
        self.create_subscription(
            Image, 
            '/left/image_raw', 
            self.image_cb, 
            qos_profile=qos_profile_sensor_data
        )
        self.create_subscription(
            Detection2DArray, 
            '/left/detections', 
            self.det_cb, 
            qos_profile=qos_profile_sensor_data
        )
        
        self.pub = self.create_publisher(CompressedImage, '/left/compressed_visual_debug', 10)
        self.latest_detections = []
        self.get_logger().info("Headless Visualizer Node Started. Waiting for images...")

    def det_cb(self, msg):
        self.latest_detections = msg.detections

    def image_cb(self, msg):
        try:
            # Isaac ROS typically outputs 'rgb8' or 'bgr8'
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        orig_h, orig_w = cv_img.shape[:2]

        # Scale down image to reduce bandwidth
        target_w = 640
        target_h = int(orig_h * (target_w / float(orig_w)))
        scale_x = target_w / float(orig_w)
        scale_y = target_h / float(orig_h)

        cv_img_resized = cv2.resize(cv_img, (target_w, target_h))

        # Draw bounding boxes
        for det in self.latest_detections:
            if not det.results:
                continue
            class_id = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score

            cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
            w, h = det.bbox.size_x, det.bbox.size_y

            x1 = int((cx - w/2) * scale_x)
            y1 = int((cy - h/2) * scale_y)
            x2 = int((cx + w/2) * scale_x)
            y2 = int((cy + h/2) * scale_y)

            cv2.rectangle(cv_img_resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"ID {class_id}: {score:.2f}"
            cv2.putText(cv_img_resized, label, (x1, max(y1 - 5, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Compress to JPEG
        success, encoded_img = cv2.imencode('.jpg', cv_img_resized, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not success:
            return

        comp_msg = CompressedImage()
        comp_msg.header = msg.header
        comp_msg.format = "jpeg"
        comp_msg.data = encoded_img.tobytes()

        self.pub.publish(comp_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HeadlessVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()