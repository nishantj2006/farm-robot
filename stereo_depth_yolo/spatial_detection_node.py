#!/usr/bin/env python3
import math
import cv2
import numpy as np
import time

import rclpy
from rclpy.node import Node
import message_filters
from cv_bridge import CvBridge

from vision_msgs.msg import Detection2DArray, Detection3DArray, Detection3D, ObjectHypothesisWithPose
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray

class SpatialDetectionNode(Node):
    def __init__(self):
        super().__init__('spatial_detection_node')
        
        self.declare_parameter('pad_y', 8.0)
        self.pad_y = float(self.get_parameter('pad_y').value)

        self.bridge = CvBridge()

        # --- Diagnostic Stream Counters ---
        self.rx_disp = 0
        self.rx_det = 0
        self.rx_img = 0
        self.rx_info = 0

        self.frame_count = 0
        self.start_time = time.time()

        # 1. Independent Diagnostic Subscribers (Bypasses Synchronizer to isolate broken topics)
        self.create_subscription(DisparityImage, '/disparity', self._diag_disp, 10)
        self.create_subscription(Detection2DArray, '/detections_output', self._diag_det, 10)
        self.create_subscription(Image, '/left/image_rect', self._diag_img, 10)
        self.create_subscription(CameraInfo, '/left/camera_info_rect', self._diag_info, 10)

        # 2. Synchronized Subscriptions via Message Filters
        self.disparity_sub = message_filters.Subscriber(self, DisparityImage, '/disparity')
        self.detections_sub = message_filters.Subscriber(self, Detection2DArray, '/detections_output')
        self.image_sub = message_filters.Subscriber(self, Image, '/left/image_rect')
        self.camera_info_sub = message_filters.Subscriber(self, CameraInfo, '/left/camera_info_rect')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.disparity_sub, self.detections_sub, self.image_sub, self.camera_info_sub],
            queue_size=30,
            slop=0.04
        )
        self.ts.registerCallback(self.synced_callback)

        # Publishers
        self.annotated_pub = self.create_publisher(Image, '/spatial/annotated_image', 10)
        self.detections_3d_pub = self.create_publisher(Detection3DArray, '/spatial/detections_3d', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/spatial/markers_3d', 10)

        # Periodic Diagnostic Timer (Runs every 2.0 seconds)
        self.diag_timer = self.create_timer(2.0, self.diagnostic_timer_cb)

        self.get_logger().info(f"[DIAGNOSTIC] 3D Spatial Node initialized. Monitoring pipeline topics...")

    # Diagnostic callbacks to count incoming frames per topic
    def _diag_disp(self, msg): self.rx_disp += 1
    def _diag_det(self, msg): self.rx_det += 1
    def _diag_img(self, msg): self.rx_img += 1
    def _diag_info(self, msg): self.rx_info += 1

    def diagnostic_timer_cb(self):
        """Prints incoming topic heartbeat independent of synchronization."""
        self.get_logger().info(
            f"\n--- TOPIC HEARTBEAT (Last 2s) ---\n"
            f" 1. /left/image_rect:      {self.rx_img} msgs\n"
            f" 2. /left/camera_info_rect: {self.rx_info} msgs\n"
            f" 3. /disparity:             {self.rx_disp} msgs\n"
            f" 4. /detections_output:     {self.rx_det} msgs\n"
            f"--------------------------------"
        )
        
        # Identify missing streams
        if self.rx_img == 0:
            self.get_logger().warn("--> STUCK AT: Rectify Node / Camera output (/left/image_rect zero msgs)")
        elif self.rx_disp == 0:
            self.get_logger().warn("--> STUCK AT: Stereo Disparity Node (/disparity zero msgs)")
        elif self.rx_det == 0:
            self.get_logger().warn("--> STUCK AT: YOLO Pipeline - Encoder/TensorRT/Decoder (/detections_output zero msgs)")
        elif self.frame_count == 0:
            self.get_logger().warn("--> STUCK AT: Time Synchronizer! Topic timestamps do not align within slop range.")

        # Reset counters for next cycle
        self.rx_disp = 0
        self.rx_det = 0
        self.rx_img = 0
        self.rx_info = 0

    def synced_callback(self, disp_msg, det_msg, img_msg, info_msg):
        self.frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        if elapsed >= 1.0:
            fps = self.frame_count / elapsed
            self.get_logger().info(f"==> Pipeline Speed: {fps:.2f} FPS <==")
            self.frame_count = 0
            self.start_time = current_time

        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            disp_img = self.bridge.imgmsg_to_cv2(disp_msg.image, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        f = disp_msg.f
        b = disp_msg.t
        min_disp = disp_msg.min_disparity
        max_disp = disp_msg.max_disparity

        cx_cam = info_msg.k[2] if info_msg.k[2] != 0.0 else disp_img.shape[1] / 2.0
        cy_cam = info_msg.k[5] if info_msg.k[5] != 0.0 else disp_img.shape[0] / 2.0

        detections_3d_msg = Detection3DArray()
        detections_3d_msg.header = img_msg.header
        marker_array = MarkerArray()

        for idx, det in enumerate(det_msg.detections):
            u = float(det.bbox.center.position.x)
            v = float(det.bbox.center.position.y) - self.pad_y
            w = int(det.bbox.size_x)
            h = int(det.bbox.size_y)

            cx, cy = int(u), int(v)

            xmin_box = max(0, cx - int(w / 2))
            xmax_box = min(disp_img.shape[1], cx + int(w / 2))
            ymin_box = max(0, cy - int(h / 2))
            ymax_box = min(disp_img.shape[0], cy + int(h / 2))

            w_offset = max(1, int(w * 0.25))
            h_offset = max(1, int(h * 0.25))

            xmin_roi = max(0, cx - w_offset)
            xmax_roi = min(disp_img.shape[1], cx + w_offset)
            ymin_roi = max(0, cy - h_offset)
            ymax_roi = min(disp_img.shape[0], cy + h_offset)

            roi = disp_img[ymin_roi:ymax_roi, xmin_roi:xmax_roi]
            if roi.size == 0:
                continue

            valid_mask = (roi > min_disp) & (roi < max_disp) & (~np.isnan(roi))
            if not np.any(valid_mask):
                continue

            d = np.median(roi[valid_mask])
            if d <= 0:
                continue

            Z = (f * b) / d
            X = ((u - cx_cam) * Z) / f
            Y = ((v - cy_cam) * Z) / f
            dist_3d = math.sqrt(X**2 + Y**2 + Z**2)

            label = det.results[0].hypothesis.class_id if det.results else "object"
            score = det.results[0].hypothesis.score if det.results else 0.0

            self.get_logger().info(f" -> [3D DET] {label} ({score:.2f}) | Dist: {dist_3d:.2f}m | X:{X:.2f} Y:{Y:.2f} Z:{Z:.2f}")

            cv2.rectangle(frame, (xmin_box, ymin_box), (xmax_box, ymax_box), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {score:.2f} | {dist_3d:.2f}m", (xmin_box, max(15, ymin_box - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            det3d = Detection3D()
            det3d.header = img_msg.header
            if det.results:
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = label
                hyp.hypothesis.score = score
                det3d.results.append(hyp)

            det3d.bbox.center.position.x = float(X)
            det3d.bbox.center.position.y = float(Y)
            det3d.bbox.center.position.z = float(Z)
            detections_3d_msg.detections.append(det3d)

            marker = Marker()
            marker.header = img_msg.header
            marker.ns = "spatial_3d"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(X)
            marker.pose.position.y = float(Y)
            marker.pose.position.z = float(Z)
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.a = 0.8
            marker_array.markers.append(marker)

        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out_msg.header = img_msg.header
        self.annotated_pub.publish(out_msg)
        self.detections_3d_pub.publish(detections_3d_msg)
        self.marker_pub.publish(marker_array)

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
