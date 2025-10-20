#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from tf2_ros import Buffer, TransformListener
import numpy as np

class DepthTFNode(Node):
    def __init__(self):
        super().__init__('depth_tf_node')
        self.bridge = CvBridge()
        self.model = PinholeCameraModel()
        self.depth = None
        self.camera_info_received = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, '/target_position', 10)

        self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.caminfo_cb, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_cb, 10)
        self.create_subscription(Detection2DArray, '/yolo/detections', self.detection_cb, 10)

        self.get_logger().info("✅ Depth + TF Node Ready")

    def caminfo_cb(self, msg):
        if not self.camera_info_received:
            self.model.fromCameraInfo(msg)
            self.camera_info_received = True
            self.camera_frame = msg.header.frame_id

    def depth_cb(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def detection_cb(self, msg):
        if not self.camera_info_received or self.depth is None:
            return
        if len(msg.detections) == 0:
            return

        # 最もスコアの高い検出を使用
        det = max(msg.detections, key=lambda d: d.results[0].hypothesis.score)
        cx, cy = int(det.bbox.center.x), int(det.bbox.center.y)
        h, w = self.depth.shape

        # 深度取得
        x0, x1 = max(0, cx - 1), min(w, cx + 2)
        y0, y1 = max(0, cy - 1), min(h, cy + 2)
        depth_vals = self.depth[y0:y1, x0:x1].astype(np.float32)
        valid = depth_vals[(depth_vals > 0) & np.isfinite(depth_vals)]
        if valid.size == 0:
            self.get_logger().warn("No valid depth value.")
            return
        depth_m = np.median(valid) / 1000.0

        # カメラ座標系→3D座標
        ray = np.array(self.model.projectPixelTo3dRay((cx, cy)))
        ray /= np.linalg.norm(ray)
        pos_cam = ray * depth_m

        pose = PoseStamped()
        pose.header.frame_id = self.camera_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pos_cam.tolist()
        pose.pose.orientation.w = 1.0

        try:
            pose_tf = self.tf_buffer.transform(pose, 'base_link', timeout=Duration(seconds=0.3))
            self.pose_pub.publish(pose_tf)
            self.get_logger().info(f"→ Published: x={pose_tf.pose.position.x:.3f}, y={pose_tf.pose.position.y:.3f}, z={pose_tf.pose.position.z:.3f}")
        except Exception as e:
            self.get_logger().warn(f"TF failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthTFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
