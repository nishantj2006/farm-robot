#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection3DArray, Detection3D, ObjectHypothesisWithPose
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class SpatialDetectionNode(Node):
    def __init__(self):
        super().__init__('spatial_detection_node')
        
        self.bridge = CvBridge()
        self.latest_disparity_msg = None
        self.latest_detections = []
        self.camera_info = None

        # Subscriptions
        self.create_subscription(DisparityImage, '/disparity', self.disparity_cb, 10)
        self.create_subscription(Detection2DArray, '/detections_output', self.detections_cb, 10)
        self.create_subscription(Image, '/left/image_rect', self.image_cb, 10)
        self.create_subscription(CameraInfo, '/left/camera_info', self.camera_info_cb, 10)

        # Publishers
        self.annotated_pub = self.create_publisher(Image, '/spatial/annotated_image', 10)
        self.detections_3d_pub = self.create_publisher(Detection3DArray, '/spatial/detections_3d', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/spatial/markers_3d', 10)

        self.get_logger().info("3D Spatial Detection Node Initialized. Streaming feed...")

    def camera_info_cb(self, msg):
        self.camera_info = msg

    def disparity_cb(self, msg):
        self.latest_disparity_msg = msg

    def detections_cb(self, msg):
        self.latest_detections = msg.detections

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        detections_3d_msg = Detection3DArray()
        detections_3d_msg.header = msg.header
        marker_array = MarkerArray()

        if self.latest_disparity_msg is not None and self.latest_detections:
            try:
                disp_img = self.bridge.imgmsg_to_cv2(self.latest_disparity_msg.image, desired_encoding='passthrough')
                f = self.latest_disparity_msg.f
                b = self.latest_disparity_msg.t
                min_disp = self.latest_disparity_msg.min_disparity
                max_disp = self.latest_disparity_msg.max_disparity

                if self.camera_info is not None:
                    cx_cam = self.camera_info.k[2]
                    cy_cam = self.camera_info.k[5]
                else:
                    cx_cam = disp_img.shape[1] / 2.0
                    cy_cam = disp_img.shape[0] / 2.0

                for idx, det in enumerate(self.latest_detections):
                    u = float(det.bbox.center.position.x)
                    v = float(det.bbox.center.position.y)
                    w = int(det.bbox.size_x)
                    h = int(det.bbox.size_y)

                    cx = int(u)
                    cy = int(v)

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

                    # --- 3D Spatial Calculation ---
                    Z = (f * b) / d
                    X = ((u - cx_cam) * Z) / f
                    Y = ((v - cy_cam) * Z) / f
                    dist_3d = math.sqrt(X**2 + Y**2 + Z**2)

                    label = det.results[0].hypothesis.class_id if det.results else "lemon"
                    score = det.results[0].hypothesis.score if det.results else 0.0

                    self.get_logger().info(
                        f"Detected '{label}' ({score:.2f}) at Spatial Position: "
                        f"X:{X:.2f}m, Y:{Y:.2f}m, Z:{Z:.2f}m (3D Distance: {dist_3d:.2f}m)"
                    )

                    # --- Visual Overlays ---
                    cv2.rectangle(frame, (xmin_box, ymin_box), (xmax_box, ymax_box), (0, 255, 0), 2)
                    
                    text_line1 = f"{label} {score:.2f} | Dist:{dist_3d:.2f}m"
                    text_line2 = f"X:{X:.2f} Y:{Y:.2f} Z:{Z:.2f}m"

                    cv2.rectangle(frame, (xmin_box, max(0, ymin_box - 40)), (xmin_box + 260, ymin_box), (0, 0, 0), -1)
                    cv2.putText(frame, text_line1, (xmin_box + 4, max(15, ymin_box - 22)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(frame, text_line2, (xmin_box + 4, max(30, ymin_box - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                    # --- Construct 3D Messages ---
                    det3d = Detection3D()
                    det3d.header = msg.header
                    if det.results:
                        hyp = ObjectHypothesisWithPose()
                        hyp.hypothesis.class_id = label
                        hyp.hypothesis.score = score
                        det3d.results.append(hyp)

                    det3d.bbox.center.position.x = float(X)
                    det3d.bbox.center.position.y = float(Y)
                    det3d.bbox.center.position.z = float(Z)
                    det3d.bbox.size.x = (w * Z) / f
                    det3d.bbox.size.y = (h * Z) / f
                    det3d.bbox.size.z = 0.1
                    detections_3d_msg.detections.append(det3d)

                    marker = Marker()
                    marker.header = msg.header
                    marker.ns = "lemons_3d"
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
                    marker.color.g = 0.9
                    marker.color.b = 0.0
                    marker.color.a = 0.8
                    marker_array.markers.append(marker)

            except Exception as e:
                self.get_logger().error(f"Error processing 3D spatial detection: {e}")

        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out_msg.header = msg.header
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
