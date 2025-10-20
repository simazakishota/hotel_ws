#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import numpy as np
import cv2
import open3d as o3d
from image_geometry import PinholeCameraModel


class AlignedGroundBoxDetector(Node):
    def __init__(self):
        super().__init__('aligned_ground_box_detector')
        self.bridge = CvBridge()
        self.model = PinholeCameraModel()
        self.camera_info_received = False

        # === サブスクライブ ===
        # カラー画像（基準）
        self.create_subscription(Image,
                                 '/camera/camera/color/image_raw',
                                 self.image_callback, 10)

        # アライン済みDepth（カラー座標に対応）
        self.create_subscription(Image,
                                 '/camera/camera/aligned_depth_to_color/image_raw',
                                 self.depth_callback, 10)

        # カメラ内部パラメータ
        self.create_subscription(CameraInfo,
                                 '/camera/camera/aligned_depth_to_color/camera_info',
                                 self.camera_info_callback, 10)

        # 出力
        self.det_pub = self.create_publisher(Detection2DArray, '/ground_box/detections', 10)

        # === 内部変数 ===
        self.latest_depth = None
        self.latest_color = None
        self.voxel_size = 0.02
        self.scan_interval = 2.0
        self.timer = self.create_timer(self.scan_interval, self.process_latest)

        cv2.namedWindow("Aligned Ground Box", cv2.WINDOW_NORMAL)
        self.get_logger().info("✅ AlignedGroundBoxDetector Ready (RGB-Depth pixel alignment active)")

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.model.fromCameraInfo(msg)
            self.camera_info_received = True
            self.camera_frame_id = msg.header.frame_id
            self.get_logger().info(f"[Camera Info] fx={self.model.fx():.1f}, fy={self.model.fy():.1f}")

    def image_callback(self, msg):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1").astype(np.float32) / 1000.0  # m単位

    def process_latest(self):
        if self.latest_depth is None or self.latest_color is None or not self.camera_info_received:
            return

        color = self.latest_color.copy()
        depth = self.latest_depth.copy()
        h, w = color.shape[:2]

        # === 点群生成 (aligned depthからOpen3Dへ) ===
        fx, fy = self.model.fx(), self.model.fy()
        cx, cy = self.model.cx(), self.model.cy()

        xs, ys = np.meshgrid(np.arange(w), np.arange(h))
        z = depth
        valid = (z > 0) & (z < 5.0)
        x = (xs - cx) * z / fx
        y = (ys - cy) * z / fy
        xyz = np.stack((x[valid], y[valid], z[valid]), axis=-1)
        if xyz.shape[0] < 1000:
            return

        # === Open3Dで地面除去 ===
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd = pcd.voxel_down_sample(self.voxel_size)

        try:
            plane_model, inliers = pcd.segment_plane(distance_threshold=0.02,
                                                     ransac_n=3,
                                                     num_iterations=300)
        except Exception:
            return

        [a, b, c, d] = plane_model
        ground_height = -d / c
        obj_cloud = pcd.select_by_index(inliers, invert=True)

        labels = np.array(obj_cloud.cluster_dbscan(eps=0.06, min_points=40))
        num_clusters = labels.max() + 1

        det_array = Detection2DArray()
        det_array.header.frame_id = self.camera_frame_id

        for i in range(num_clusters):
            cluster = obj_cloud.select_by_index(np.where(labels == i)[0])
            if len(cluster.points) < 80:
                continue

            pts = np.asarray(cluster.points)
            obb = cluster.get_oriented_bounding_box()
            w_, d_, h_ = sorted(obb.extent)

            ratio_wh = w_ / h_
            ratio_dh = d_ / h_
            if not (0.2 < ratio_wh < 1.2 and 0.2 < ratio_dh < 1.2):
                continue

            z_min = np.min(pts[:, 2])
            if abs(z_min - ground_height) > 0.05:
                continue

            cx3, cy3, cz3 = np.mean(pts, axis=0)

            # === 3D→画像座標投影 ===
            uv = self.model.project3dToPixel((cx3, cy3, cz3))
            u, v = int(uv[0]), int(uv[1])
            if 0 <= u < w and 0 <= v < h:
                cv2.circle(color, (u, v), 6, (0, 255, 0), -1)
                cv2.putText(color, f"Box{i}", (u + 10, v),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            det = Detection2D()
            det.bbox.center.position.x = cx3
            det.bbox.center.position.y = cy3
            det.bbox.size_x = w_
            det.bbox.size_y = d_

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = "ground_box"
            hyp.hypothesis.score = float(h_ / (w_ + d_))
            det.results.append(hyp)
            det_array.detections.append(det)

        if len(det_array.detections) > 0:
            self.det_pub.publish(det_array)

        cv2.imshow("Aligned Ground Box", color)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = AlignedGroundBoxDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
