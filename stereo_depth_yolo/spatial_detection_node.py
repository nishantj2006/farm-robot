#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import CameraInfo
import message_filters

class SpatialDetectionNode(Node):
    def __init__(self):
        super().__init__('spatial_detection_node')
        
        # Physical camera baseline in meters (60mm)
        self.baseline = 0.06 
        self.focal_length_x = None
        
        # 1. Grab intrinsic focal length from the Rectified Camera Info
        self.info_sub = self.create_subscription(
            CameraInfo, '/left/camera_info_rect', self.camera_info_callback, 10
        )
        
        # 2. Subscribe to both high-speed YOLO outputs
        self.left_sub = message_filters.Subscriber(self, Detection2DArray, '/left/detections')
        self.right_sub = message_filters.Subscriber(self, Detection2DArray, '/right/detections')
        
        # 3. Synchronize AI outputs (Small queue prevents memory leaks)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], queue_size=10, slop=0.05
        )
        self.sync.registerCallback(self.sync_callback)

        # self.left_sub = self.create_subscription(
        #     Detection2DArray,
        #     '/left/detections',
        #     self.single_cam_callback,
        #     10
        # )

        self.get_logger().info("Sparse Stereo Matchmaker initialized. Waiting for detections...")


    def single_cam_callback(self, msg):
        if msg.detections:
            self.get_logger().info(f"Left Camera Detected: {len(msg.detections)} object(s)!")


    def camera_info_callback(self, msg):
        if self.focal_length_x is None:
            # The Projection Matrix P contains the focal length at index 0
            self.focal_length_x = msg.p[0]
            self.get_logger().info(f"Acquired Focal Length (f): {self.focal_length_x:.2f} pixels")

    def sync_callback(self, left_msg, right_msg):
        # Don't compute anything until we have our focal length
        if self.focal_length_x is None:
            return
            
        if not left_msg.detections or not right_msg.detections:
            return

        # Iterate through all objects YOLO found in the LEFT camera
        for l_det in left_msg.detections:
            l_class = l_det.results[0].hypothesis.class_id
            l_x = l_det.bbox.center.position.x
            l_y = l_det.bbox.center.position.y
            
            best_match = None
            best_y_diff = float('inf')
            
            # Find the matching object in the RIGHT camera
            for r_det in right_msg.detections:
                r_class = r_det.results[0].hypothesis.class_id
                r_x = r_det.bbox.center.position.x
                r_y = r_det.bbox.center.position.y
                
                # Rule 1: Must be the same object class (e.g., both are Lemons)
                if l_class == r_class:
                    # Rule 2: Epipolar Constraint (Y-centers must vertically align)
                    y_diff = abs(l_y - r_y)
                    
                    # If they align within 15 pixels, and the left object is physically further right in the frame (x_L > x_R)
                    if y_diff < 15.0 and y_diff < best_y_diff and l_x > r_x:
                        best_match = r_det
                        best_y_diff = y_diff
            
            # If we found a valid stereo pair, calculate 3D distance
            if best_match:
                r_x = best_match.bbox.center.position.x
                disparity = l_x - r_x
                
                # Execute standard triangulation: Z = (f * B) / Disparity
                depth_z = (self.focal_length_x * self.baseline) / disparity
                
                self.get_logger().info(
                    f"Match Found! [Class: {l_class}] | Disparity: {disparity:.1f}px | Distance (Z): {depth_z:.2f} meters"
                )

def main(args=None):
    rclpy.init(args=args)
    node = SpatialDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()