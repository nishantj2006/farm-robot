#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import CameraInfo
import message_filters

class SpatialDetectionNode(Node):
    def __init__(self):
        super().__init__('spatial_detection_node')
        
        # Thread safety callback group
        self.cb_group = ReentrantCallbackGroup()

        # Physical camera baseline in meters (60mm)
        self.baseline = 0.06 
        self.focal_length_x = None
        self.focal_length_y = None
        self.center_x = None
        self.center_y = None
        
        # 1. Grab intrinsic focal length from Rectified Camera Info
        self.info_sub = self.create_subscription(
            CameraInfo, '/left/camera_info_rect', self.camera_info_callback, 10,
            callback_group=self.cb_group
        )
        
        # 2. Subscribe to both high-speed YOLO outputs
        self.left_sub = message_filters.Subscriber(
            self, Detection2DArray, '/left/detections', callback_group=self.cb_group
        )
        self.right_sub = message_filters.Subscriber(
            self, Detection2DArray, '/right/detections', callback_group=self.cb_group
        )
        
        # 3. Synchronize AI outputs (Increased slop to 120ms to prevent dropped pairs)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], queue_size=20, slop=0.12
        )
        self.sync.registerCallback(self.sync_callback)

        self.get_logger().info("Sparse Stereo Matchmaker initialized. Waiting for detections...")

    def camera_info_callback(self, msg):
        if self.focal_length_x is None:
            # P matrix: [f_x, 0, c_x, Tx, 0, f_y, c_y, Ty, 0, 0, 1, 0]
            self.focal_length_x = msg.p[0]  # f_x
            self.focal_length_y = msg.p[5]  # f_y
            self.center_x = msg.p[2]        # c_x
            self.center_y = msg.p[6]        # c_y
            self.get_logger().info("Acquired Camera Intrinsics for 3D Projection")
            
            # Destroy subscription after acquiring intrinsics to save CPU
            self.destroy_subscription(self.info_sub)

    def sync_callback(self, left_msg, right_msg):
        if self.focal_length_x is None:
            return

        left_count = len(left_msg.detections)
        right_count = len(right_msg.detections)

        # -------------------------------------------------------------
        # 1. Log LEFT Camera Detections
        # -------------------------------------------------------------
        for l_det in left_msg.detections:
            l_class = l_det.results[0].hypothesis.class_id
            l_x = l_det.bbox.center.position.x
            l_y = l_det.bbox.center.position.y
            self.get_logger().info(
                f"[LEFT CAM] Detected Class: {l_class} at pixel ({l_x:.1f}, {l_y:.1f})"
            )

        # -------------------------------------------------------------
        # 2. Log RIGHT Camera Detections
        # -------------------------------------------------------------
        for r_det in right_msg.detections:
            r_class = r_det.results[0].hypothesis.class_id
            r_x = r_det.bbox.center.position.x
            r_y = r_det.bbox.center.position.y
            self.get_logger().info(
                f"[RIGHT CAM] Detected Class: {r_class} at pixel ({r_x:.1f}, {r_y:.1f})"
            )

        # Explicit diagnostic log when one camera yields zero detections
        if left_count > 0 and right_count == 0:
            self.get_logger().warn("[SYNC WARNING] Left camera saw object, but Right camera output 0 detections.")
            return
        elif left_count == 0 and right_count > 0:
            self.get_logger().warn("[SYNC WARNING] Right camera saw object, but Left camera output 0 detections.")
            return
        elif left_count == 0 and right_count == 0:
            return

        # -------------------------------------------------------------
        # 3. Perform Epipolar Stereo Matching & Triangulation
        # -------------------------------------------------------------
        for l_det in left_msg.detections:
            l_class = l_det.results[0].hypothesis.class_id
            l_x = l_det.bbox.center.position.x
            l_y = l_det.bbox.center.position.y
            
            best_match = None
            best_y_diff = float('inf')
            
            for r_det in right_msg.detections:
                r_class = r_det.results[0].hypothesis.class_id
                r_x = r_det.bbox.center.position.x
                r_y = r_det.bbox.center.position.y
                
                # Rule 1: Same object class
                if l_class == r_class:
                    # Rule 2: Epipolar Constraint (Y-centers must vertically align)
                    y_diff = abs(l_y - r_y)
                    
                    # Allow up to 20px vertical tolerance and require l_x > r_x
                    if y_diff < 20.0 and y_diff < best_y_diff and l_x > r_x:
                        best_match = r_det
                        best_y_diff = y_diff
            
            # If a valid stereo pair is found, calculate 3D distance
            if best_match:
                r_x = best_match.bbox.center.position.x
                disparity = l_x - r_x
                
                if disparity > 0:
                    depth_z = (self.focal_length_x * self.baseline) / disparity
                    real_x = ((l_x - self.center_x) * depth_z) / self.focal_length_x
                    real_y = ((l_y - self.center_y) * depth_z) / self.focal_length_y
                    
                    self.get_logger().info(
                        f"[STEREO MATCH] [Class: {l_class}] | X: {real_x:.2f}m, Y: {real_y:.2f}m, Z: {depth_z:.2f}m"
                    )

def main(args=None):
    rclpy.init(args=args)
    node = SpatialDetectionNode()
    
    # Use MultiThreadedExecutor to process callbacks concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()