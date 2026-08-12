#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image, CameraInfo

class PyNitrosSyncNode(Node):
    def __init__(self):
        super().__init__('pynitros_sync_node')

        # Publishers
        self.left_img_pub = self.create_publisher(Image, '/synced/left/image_raw', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, '/synced/left/camera_info', 10)
        self.right_img_pub = self.create_publisher(Image, '/synced/right/image_raw', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/synced/right/camera_info', 10)

        # Subscribers
        self.left_img_sub = message_filters.Subscriber(self, Image, '/left/image_raw')
        self.left_info_sub = message_filters.Subscriber(self, CameraInfo, '/left/camera_info')
        self.right_img_sub = message_filters.Subscriber(self, Image, '/right/image_raw')
        self.right_info_sub = message_filters.Subscriber(self, CameraInfo, '/right/camera_info')

        # Approximate synchronizer to catch the drifting hardware timestamps
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_img_sub, self.left_info_sub, self.right_img_sub, self.right_info_sub],
            queue_size=10,
            slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)
        self.get_logger().info("Python Zero-Copy Sync Node Initialized.")

    def sync_callback(self, l_img, l_info, r_img, r_info):
        # 1. Grab the unified timestamp from the left camera
        unified_stamp = l_img.header.stamp

        # 2. Mutate the headers (Python passes by reference, bypassing the data deep-copy)
        l_img.header.stamp = unified_stamp
        l_info.header.stamp = unified_stamp
        r_img.header.stamp = unified_stamp
        r_info.header.stamp = unified_stamp

        # 3. Publish directly to the Rectify nodes
        self.left_img_pub.publish(l_img)
        self.left_info_pub.publish(l_info)
        self.right_img_pub.publish(r_img)
        self.right_info_pub.publish(r_info)

def main(args=None):
    rclpy.init(args=args)
    node = PyNitrosSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()