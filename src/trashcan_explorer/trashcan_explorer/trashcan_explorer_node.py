#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped, PoseStamped
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import math
import time
import tf2_geometry_msgs


class TrashcanExplorer(Node):
    def __init__(self):
        super().__init__('trashcan_explorer_node')

        # ===== Publisher =====
        self.pub_candidates = self.create_publisher(PoseStamped, '/trashcan_candidates', 10)

        # ===== TF設定 =====
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ===== YOLOモデル =====
        self.model1 = YOLO(r"/home/araishogo/hotel_ws/src/best (8).pt")
        self.model2 = YOLO(r"/home/araishogo/hotel_ws/src/best2.pt")

        # ===== RealSense初期化 =====
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.align = rs.align(rs.stream.color)

        self.get_logger().info("📷 RealSense起動中...")
        self.pipeline.start(self.config)
        for _ in range(30):
            self.pipeline.wait_for_frames()

        self.capture_and_detect()

    # ===== TF待機 =====
    def wait_for_tf(self, target='base_link', source='camera_camera_camera_link', timeout_sec=5.0):
        start = time.time()
        while rclpy.ok():
            if self.tf_buffer.can_transform(target, source, rclpy.time.Time()):
                return True
            if time.time() - start > timeout_sec:
                self.get_logger().warn("⚠️ TF受信タイムアウト")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        return False

    # ===== カメラロール角取得 =====
    def get_camera_roll(self):
        try:
            trans = self.tf_buffer.lookup_transform('base_link', 'camera_camera_camera_link', rclpy.time.Time())
            q = trans.transform.rotation
            sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
            cosr_cosp = 1.0 - 2.0 * (q.x**2 + q.y**2)
            roll = math.pi + math.atan2(sinr_cosp, cosr_cosp)
            self.get_logger().info(f"📐 カメラロール: {math.degrees(roll):.2f}°")
            return roll
        except Exception as e:
            self.get_logger().warn(f"⚠️ ロール角取得失敗: {e}")
            return 0.0

    # ===== bbox中心をBaseLinkへ変換 =====
    def project_to_baselink(self, cx, cy, depth, intrinsics):
        X = (cx - intrinsics.ppx) * depth / intrinsics.fx
        Y = (cy - intrinsics.ppy) * depth / intrinsics.fy
        Z = depth

        pt_cam = PointStamped()
        pt_cam.header.frame_id = 'camera_camera_camera_link'
        pt_cam.header.stamp = rclpy.time.Time().to_msg()
        pt_cam.point.x = X
        pt_cam.point.y = Y
        pt_cam.point.z = Z

        try:
            pt_base = self.tf_buffer.transform(pt_cam, 'base_link', timeout=rclpy.duration.Duration(seconds=1.0))
            return pt_base.point.x, pt_base.point.y, pt_base.point.z
        except Exception as e:
            self.get_logger().warn(f"⚠️ TF変換失敗: {e}")
            return None

    # ===== メイン処理 =====
    def capture_and_detect(self):
        try:
            frames = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                raise RuntimeError("❌ フレーム取得失敗")

            color_img = np.asanyarray(color_frame.get_data())
            intr = depth_frame.profile.as_video_stream_profile().get_intrinsics()
            fx, fy = intr.fx, intr.fy

            self.get_logger().info("🧠 YOLO第1段推論中...")
            res1 = self.model1.predict(source=color_img, conf=0.2, verbose=False)[0]
            canvas = color_img.copy()

            if not self.wait_for_tf():
                self.get_logger().warn("⚠️ TF未取得のため変換スキップ")

            roll = self.get_camera_roll()  # カメラロール角（ラジアン）

            final_candidates = []  # フィルタを通過した候補リスト

            for b in res1.boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                crop = color_img[y1:y2, x1:x2]
                if crop.size == 0:
                    continue

                res2 = self.model2.predict(source=crop, conf=0.1, verbose=False)[0]
                if len(res2.boxes) == 0:
                    continue

                best_box = res2.boxes[0]
                xx1, yy1, xx2, yy2 = map(int, best_box.xyxy[0].tolist())
                gx1, gy1, gx2, gy2 = x1 + xx1, y1 + yy1, x1 + xx2, y1 + yy2
                cx, cy = int((gx1 + gx2) / 2), int((gy1 + gy2) / 2)
                depth = depth_frame.get_distance(cx, cy)
                if depth == 0:
                    continue

                # ==== 実寸高さ計算 ====
                pixel_height = gy2 - gy1
                pixel_width = gx2 - gx1
                pixel_to_meter = depth / fy
                real_height = pixel_height * pixel_to_meter * abs(math.sin(roll))

                # ==== フィルタ条件 ====
                height_ok = 0.25 <= real_height <= 0.4
                shape_ok = pixel_height > pixel_width

                if height_ok and shape_ok:
                    base_coords = self.project_to_baselink(cx, cy, depth, intr)
                    if base_coords:
                        bx, by, bz = base_coords
                        final_candidates.append((gx1, gy1, gx2, gy2, bx, by, bz, real_height))

                        # === トピック出力 ===
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = self.get_clock().now().to_msg()
                        pose_msg.header.frame_id = "base_link"
                        pose_msg.pose.position.x = bx
                        pose_msg.pose.position.y = by
                        pose_msg.pose.position.z = bz
                        pose_msg.pose.orientation.w = 1.0
                        self.pub_candidates.publish(pose_msg)
                        self.get_logger().info(f"📡 Publish候補: ({bx:.2f}, {by:.2f}, {bz:.2f})")

            # ==== フィルタ結果の可視化（保持） ====
            if len(final_candidates) == 0:
                self.get_logger().info("🚫 該当する候補なし")
            else:
                for gx1, gy1, gx2, gy2, bx, by, bz, real_height in final_candidates:
                    label = f"({bx:.2f}, {by:.2f})m  H={real_height:.2f}m"
                    cv2.putText(canvas, label, (gx1, gy1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                    cv2.rectangle(canvas, (gx1, gy1), (gx2, gy2), (0, 255, 255), 2)
                    self.get_logger().info(
                        f"🎯 最終候補: x={bx:.3f}, y={by:.3f}, z={bz:.3f}, 高さ={real_height:.3f}m"
                    )

                # ==== 表示を保持 ====
                scale = 640 / canvas.shape[1]
                resized = cv2.resize(canvas, None, fx=scale, fy=scale)
                cv2.imshow("YOLO Final Candidates (with Publish)", resized)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

        finally:
            self.pipeline.stop()
            self.get_logger().info("🛑 RealSenseストリーム停止")


def main(args=None):
    rclpy.init(args=args)
    node = TrashcanExplorer()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
