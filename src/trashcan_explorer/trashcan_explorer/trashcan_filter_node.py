#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
import math
import subprocess
import time


class TrashcanFilter(Node):
    def __init__(self):
        super().__init__('trashcan_filter_node')

        # ===== Subscriber / Publisher =====
        self.sub = self.create_subscription(PoseStamped, '/trashcan_candidates', self.callback, 10)
        self.pub_target = self.create_publisher(PointStamped, '/target_in_camera', 10)  # base_linkでpub

        # ===== パラメータ =====
        self.candidates = []
        self.confirmed = False
        self.distance_thresh = 0.3  # [m]
        self.min_points = 1       # 3点で確定

        # ===== 撮影姿勢 =====
        self.arm_poses = {
            0: [-0.1, 0.3, -0.5, 0.0, 0.6, 0.3, 0.0],
            1: [-0.5, 0.3, -0.5, 0.0, 0.6, 0.3, 0.0],
            2: [-0.9, 0.3, -0.5, 0.0, 0.6, 0.3, 0.0],
            3: [0.1, 0.3, -0.5, 0.0, 0.6, 0.3, 0.0],
            4: [0.5, 0.3, -0.5, 0.0, 0.6, 0.3, 0.0],
            5: [0.9, 0.3, -0.5, 0.0, 0.6, 0.3, 0.0],
        }

        self.get_logger().info("🧩 TrashcanFilterノード起動: 6姿勢で探索開始")

    # ===== Piperアームを姿勢へ移動 =====
    def move_arm(self, pos):
        cmd = [
            "ros2", "topic", "pub", "--once",
            "/joint_ctrl_single", "sensor_msgs/msg/JointState",
            f"{{header: {{stamp: {{sec: 0, nanosec: 0}}, frame_id: 'piper_single'}}, "
            f"name: ['joint1','joint2','joint3','joint4','joint5','joint6','joint7'], "
            f"position: {pos}, velocity: [0,0,0,0,0,0,10], effort: [0,0,0,0,0,0,0.5]}}"
        ]
        self.get_logger().info(f"🤖 Piperアームを姿勢へ移動中: {pos}")
        subprocess.run(cmd)
        time.sleep(2.0)

    # ===== trashcan_explorerを6姿勢で実行 =====
    def run_explorers(self):
        while not self.confirmed and rclpy.ok():
            for pose_idx, pos in self.arm_poses.items():
                if self.confirmed or not rclpy.ok():
                    break

                self.move_arm(pos)
                self.get_logger().info(f"📸 観測 {pose_idx+1}/6 実行中...")
                subprocess.run([
                    "ros2", "run", "trashcan_explorer", "trashcan_explorer_node",
                    "--ros-args", "-p", f"pose_index:={pose_idx}"
                ])
                time.sleep(2.0)

                if self.confirmed:
                    break

            if not self.confirmed:
                self.get_logger().info("🔁 6姿勢完了。確定なし。再試行...")

        if not self.confirmed:
            self.get_logger().info("🚫 探索完了: 確定候補なし。")
            rclpy.shutdown()

    # ===== 候補受信 =====
    def callback(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        self.candidates.append((x, y))
        self.get_logger().info(f"📥 受信候補: ({x:.2f}, {y:.2f})")

        clusters = self.cluster_points(self.candidates, self.distance_thresh)
        self.get_logger().info(f"📊 クラスタ数={len(clusters)} 内容={[len(c) for c in clusters]}")

        for c in clusters:
            if len(c) >= self.min_points:
                avg_x = sum(p[0] for p in c) / len(c)
                avg_y = sum(p[1] for p in c) / len(c)
                self.publish_confirmed(avg_x, avg_y)
                self.confirmed = True
                self.get_logger().info("✅ 3点検出！探索完了。")
                return

    # ===== 改良クラスタリング =====
    def cluster_points(self, points, radius):
        clusters = []
        for p in points:
            added = False
            for c in clusters:
                if any(math.hypot(px - p[0], py - p[1]) < radius for px, py in c):
                    c.append(p)
                    added = True
                    break
            if not added:
                clusters.append([p])
        return clusters

    # ===== Nav2ゴール送信 =====
    def publish_confirmed(self, x, y, z=0.0):
        # --- 初期姿勢に戻す ---
        init_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd = [
            "ros2", "topic", "pub", "--once",
            "/joint_ctrl_single", "sensor_msgs/msg/JointState",
            f"{{header: {{stamp: {{sec: 0, nanosec: 0}}, frame_id: 'piper_single'}}, "
            f"name: ['joint1','joint2','joint3','joint4','joint5','joint6','joint7'], "
            f"position: {init_pose}, velocity: [0,0,0,0,0,0,10], effort: [0,0,0,0,0,0,0.5]}}"
        ]
        self.get_logger().info("🤖 アームを初期姿勢に戻します...")
        subprocess.run(cmd)
        time.sleep(3.0)

        # --- base_link座標でそのままpub ---
        msg = PointStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z

        time.sleep(0.3)
        self.pub_target.publish(msg)
        self.get_logger().info(
            f"📡 Published /target_in_camera (base_link座標): ({x:.2f}, {y:.2f}, {z:.2f})"
        )

        time.sleep(2.0)
        self.confirmed = True


def main(args=None):
    rclpy.init(args=args)
    node = TrashcanFilter()

    import threading
    thread = threading.Thread(target=node.run_explorers, daemon=True)
    thread.start()

    while rclpy.ok() and not node.confirmed:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info("🛑 TrashcanFilterノードを終了します。")
    node.destroy_node()
    rclpy.shutdown()
