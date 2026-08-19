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


        # Calibrated 720p physical baseline (59.52 mm)
        self.baseline = 0.05952 
        
        # HARDCODED INTRINSICS FROM left2.yaml
        self.focal_length_x = 893.74551
        self.focal_length_y = 893.74551
        self.center_x = 657.54638
        self.center_y = 373.61695
        # Calibrated 720p physical baseline (59.52 mm)
        # self.baseline = 0.05952 
        # self.focal_length_x = None
        # self.focal_length_y = None
        # self.center_x = None
        # self.center_y = None
        
        # 1. Grab intrinsic focal length from Rectified Camera Info
        # self.info_sub = self.create_subscription(
        #     CameraInfo, '/left/camera_info_rect', self.camera_info_callback, 10,
        #     callback_group=self.cb_group
        # )
        
        # 2. Subscribe to both high-speed YOLO outputs
        self.left_sub = message_filters.Subscriber(
            self, Detection2DArray, '/left/detections', callback_group=self.cb_group
        )
        self.right_sub = message_filters.Subscriber(
            self, Detection2DArray, '/right/detections', callback_group=self.cb_group
        )
        
        # 3. Synchronize AI outputs (120ms slop)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], queue_size=20, slop=0.12
        )
        self.sync.registerCallback(self.sync_callback)

        self.get_logger().info("Sparse Stereo Matchmaker initialized. Waiting for detections...")

    # def camera_info_callback(self, msg):
    #     if self.focal_length_x is None:
    #         # P matrix: [f_x, 0, c_x, Tx, 0, f_y, c_y, Ty, 0, 0, 1, 0]
    #         self.focal_length_x = msg.p[0]  # f_x
    #         self.focal_length_y = msg.p[5]  # f_y
    #         self.center_x = msg.p[2]        # c_x
    #         self.center_y = msg.p[6]        # c_y
            
    #         # DIAGNOSTIC LOG: Print parameters so you can verify the YAML values
    #         self.get_logger().info(
    #             f"Acquired Intrinsics -> fx: {self.focal_length_x:.2f}, fy: {self.focal_length_y:.2f}, "
    #             f"cx: {self.center_x:.2f}, cy: {self.center_y:.2f}"
    #         )
            
    #         # Destroy subscription after acquiring intrinsics to save CPU
    #         self.destroy_subscription(self.info_sub)

    def sync_callback(self, left_msg, right_msg):
        if self.focal_length_x is None:
            return

        left_count = len(left_msg.detections)
        right_count = len(right_msg.detections)

        # -------------------------------------------------------------
        # 0. SILENT FILTER: Exit immediately if both see 0 lemons
        # -------------------------------------------------------------
        if left_count == 0 and right_count == 0:
            return

        # -------------------------------------------------------------
        # 1. Print Summary 
        # -------------------------------------------------------------
        self.get_logger().info(
            f"[FRAME SUMMARY] Left Camera sees {left_count} lemon(s) | Right Camera sees {right_count} lemon(s)"
        )

        if left_count > 0 and right_count == 0:
            self.get_logger().warn("[SYNC WARNING] Left camera saw object, but Right camera output 0 detections.")
            return
        elif left_count == 0 and right_count > 0:
            self.get_logger().warn("[SYNC WARNING] Right camera saw object, but Left camera output 0 detections.")
            return

        # -------------------------------------------------------------
        # 2. Log 2D Pixel Positions for Debugging
        # -------------------------------------------------------------
        for l_det in left_msg.detections:
            l_class = l_det.results[0].hypothesis.class_id
            l_x = l_det.bbox.center.position.x
            l_y = l_det.bbox.center.position.y
            self.get_logger().info(
                f"  -> [LEFT DET] Class {l_class} @ 2D Pixel: ({l_x:.1f}, {l_y:.1f})"
            )

        for r_det in right_msg.detections:
            r_class = r_det.results[0].hypothesis.class_id
            r_x = r_det.bbox.center.position.x
            r_y = r_det.bbox.center.position.y
            self.get_logger().info(
                f"  -> [RIGHT DET] Class {r_class} @ 2D Pixel: ({r_x:.1f}, {r_y:.1f})"
            )

        # -------------------------------------------------------------
        # 3. Robust Stereo Matching
        # -------------------------------------------------------------
        matched_pairs = 0
        claimed_r_indices = set()

        for l_det in left_msg.detections:
            raw_l_class = l_det.results[0].hypothesis.class_id
            l_class = 0 if raw_l_class in [0, 1] else raw_l_class
            
            l_x = l_det.bbox.center.position.x
            l_y = l_det.bbox.center.position.y
            l_area = l_det.bbox.size_x * l_det.bbox.size_y
            
            best_match = None
            best_match_idx = -1
            lowest_cost = float('inf')
            
            for r_idx, r_det in enumerate(right_msg.detections):
                if r_idx in claimed_r_indices:
                    continue
                
                raw_r_class = r_det.results[0].hypothesis.class_id
                r_class = 0 if raw_r_class in [0, 1] else raw_r_class
                
                r_x = r_det.bbox.center.position.x
                r_y = r_det.bbox.center.position.y
                r_area = r_det.bbox.size_x * r_det.bbox.size_y
                
                if l_class == r_class:
                    y_diff = abs(l_y - r_y)
                    
                    # Relaxed vertical tolerance (65px) to accommodate non-rectified bounding box drift
                    if y_diff < 65.0 and l_x > r_x:
                        
                        size_diff_ratio = abs(l_area - r_area) / max(l_area, r_area)
                        cost = y_diff + (size_diff_ratio * 100)
                        
                        if cost < lowest_cost and size_diff_ratio < 0.40:
                            lowest_cost = cost
                            best_match = r_det
                            best_match_idx = r_idx
            
            if best_match:
                claimed_r_indices.add(best_match_idx)
                
                r_x = best_match.bbox.center.position.x
                disparity = l_x - r_x
                
                if disparity > 0:
                    depth_z = (self.focal_length_x * self.baseline) / disparity
                    real_x = ((l_x - self.center_x) * depth_z) / self.focal_length_x
                    real_y = ((l_y - self.center_y) * depth_z) / self.focal_length_y
                    matched_pairs += 1
                    
                    self.get_logger().info(
                        f"  ==> [3D MATCH #{matched_pairs}] [Class: {l_class}] "
                        f"Location -> X: {real_x:.2f}m, Y: {real_y:.2f}m, Z: {depth_z:.2f}m"
                    )

def main(args=None):
    rclpy.init(args=args)
    node = SpatialDetectionNode()
    
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