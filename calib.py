import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class CalibCapture(Node):
    def __init__(self):
        super().__init__('calib_capture')
        self.bridge = CvBridge()
        self.img_l = None
        self.img_r = None
        self.count = 0

        # Create save folders
        os.makedirs("calib_images/left", exist_ok=True)
        os.makedirs("calib_images/right", exist_ok=True)

        # Subscribe directly to the working Isaac ROS topics
        self.create_subscription(Image, '/left/image_raw', self.cb_l, qos_profile_sensor_data)
        self.create_subscription(Image, '/right/image_raw', self.cb_r, qos_profile_sensor_data)

        # Timer to handle the GUI and saving
        self.timer = self.create_timer(0.05, self.display_and_save)
        self.get_logger().info("Listening to Isaac ROS camera streams...")

    def cb_l(self, msg):
        self.img_l = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def cb_r(self, msg):
        self.img_r = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def display_and_save(self):
        # Only update GUI if we are receiving frames from both cameras
        if self.img_l is not None and self.img_r is not None:
            cv2.imshow("Left Camera", self.img_l)
            cv2.imshow("Right Camera", self.img_r)

            key = cv2.waitKey(1) & 0xFF
            if key == 32:  # Spacebar
                cv2.imwrite(f"calib_images/left/img_{self.count:03d}.jpg", self.img_l)
                cv2.imwrite(f"calib_images/right/img_{self.count:03d}.jpg", self.img_r)
                print(f"Saved synchronized pair {self.count:03d}")
                self.count += 1
            elif key == 27:  # ESC
                rclpy.shutdown()

def main():
    rclpy.init()
    node = CalibCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
