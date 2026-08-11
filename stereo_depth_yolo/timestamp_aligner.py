#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image, CameraInfo

class TimestampAligner(Node):
    def __init__(self):
        super().__init__('timestamp_aligner')

        # Subscriptions from Rectify Nodes
        self.left_img_sub = message_filters.Subscriber(self, Image, '/left/image_rect_raw')
        self.left_info_sub = message_filters.Subscriber(self, CameraInfo, '/left/camera_info_rect_raw')
        self.right_img_sub = message_filters.Subscriber(self, Image, '/right/image_rect_raw')
        self.right_info_sub = message_filters.Subscriber(self, CameraInfo, '/right/camera_info_rect_raw')

        # Approximate Time Sync (30 ms window)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_img_sub, self.left_info_sub, self.right_img_sub, self.right_info_sub],
            queue_size=10,
            slop=0.03
        )
        self.ts.registerCallback(self.sync_callback)

        # Synchronized Publishers for Disparity Node
        self.l_img_pub = self.create_publisher(Image, '/left/image_rect', 10)
        self.l_info_pub = self.create_publisher(CameraInfo, '/left/camera_info_rect', 10)
        self.r_img_pub = self.create_publisher(Image, '/right/image_rect', 10)
        self.r_info_pub = self.create_publisher(CameraInfo, '/right/camera_info_rect', 10)

    def sync_callback(self, l_img, l_info, r_img, r_info):
        # Force identical timestamps across all 4 headers
        stamp = l_img.header.stamp
        l_info.header.stamp = stamp
        r_img.header.stamp = stamp
        r_info.header.stamp = stamp

        self.l_img_pub.publish(l_img)
        self.l_info_pub.publish(l_info)
        self.r_img_pub.publish(r_img)
        self.r_info_pub.publish(r_info)

def main(args=None):
    rclpy.init(args=args)
    node = TimestampAligner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
