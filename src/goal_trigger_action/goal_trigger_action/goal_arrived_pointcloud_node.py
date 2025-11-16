#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState, PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy
import subprocess
import signal
import time
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import os
import open3d as o3d
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
import tf_transformations
from tf_transformations import quaternion_from_matrix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class GoalArrivedPointcloudNode(Node):
    def __init__(self):
        super().__init__('goal_arrived_pointcloud_node')

        # --- /nav_goal_done 購読 ---
        self.create_subscription(Bool, '/nav_goal_done', self.goal_done_callback, 10)

        # --- Piperジョイント制御Publisher ---
        self.joint_pub = self.create_publisher(JointState, '/joint_ctrl_single', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/target_position', 10)
        # --- Gripper制御Publisher ---
        self.gripper_pub = self.create_publisher(String, '/gripper_cmd', 10)



        # --- 状態変数 ---
        self.realsense_proc = None
        self.pc_received = False
        self.pc_sub = None
        self.triggered = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("📡 GoalArrivedPointcloudNode 起動完了。Nav完了信号待機中...")

    # ===============================
    # 🚦 ゴール完了信号受信時
    # ===============================
    def goal_done_callback(self, msg):
        if msg.data and not self.triggered:
            self.triggered = True
            self.get_logger().info("🎯 ゴール到達！RealSenseを起動 → Piper姿勢へ → 点群取得します。")

            # RealSense起動
            self.start_realsense()
            time.sleep(5.0)  # 起動安定化待ち

            # Piperを所定の姿勢へ
            self.move_piper_pose()
            time.sleep(3.0)

            # 点群取得
            self.capture_pointcloud()

    # ===============================
    # ▶ RealSenseノード起動
    # ===============================
    def start_realsense(self):
        try:
            self.realsense_proc = subprocess.Popen([
                'ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
                'align_depth.enable:=true',
                'pointcloud.enable:=true',
                'depth_module.enable:=true',
                'rgb_camera.enable:=true',
                'base_frame_id:=camera_camera_link'
            ])
            self.get_logger().info("🚀 RealSenseノードを起動しました。")
        except Exception as e:
            self.get_logger().error(f"❌ RealSense起動失敗: {e}")

    # ===============================
    # ⏹ RealSenseノード停止
    # ===============================
    def stop_realsense(self):
        if self.realsense_proc:
            self.get_logger().info("🛑 RealSenseノードを停止中...")
            self.realsense_proc.send_signal(signal.SIGINT)
            self.realsense_proc.wait(timeout=10)
            self.get_logger().info("✅ RealSenseノード停止完了。")

    # ===============================
    # 🤖 Piper姿勢制御
    # ===============================
    def move_piper_pose(self):
        pose = JointState()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "piper_single"
        pose.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        pose.position = [0.0,0.6,-0.5,0.0,0.9,-2.8,0.04]
        pose.velocity = [20.0] * 7
        pose.effort = [0.0] * 7
        self.joint_pub.publish(pose)
        self.get_logger().info("🤖 Piperアームを観測姿勢へ移動中...")

    # ===============================
    # 📸 点群購読・保存・可視化
    # ===============================
    def capture_pointcloud(self):
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            qos
        )
        self.get_logger().info("📡 点群購読開始 (/camera/camera/depth/color/points)...")

        # タイムアウト待ち
        start_time = time.time()
        while not self.pc_received and (time.time() - start_time) < 10.0:
            rclpy.spin_once(self, timeout_sec=0.5)

        if not self.pc_received:
            self.get_logger().warn("⚠️ 点群を受信できませんでした（タイムアウト）")

        self.stop_realsense()

    # ===============================
    # 🧩 点群コールバック
    # ===============================

    def pointcloud_callback(self, msg):
        if self.pc_received:
            return
        self.pc_received = True

        # ===== PointCloud2 → numpy変換 =====
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array([[p[0], p[1], p[2]] for p in gen], dtype=np.float64)

        # ===== Open3D点群作成 =====
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.paint_uniform_color([0.2, 0.6, 1.0])

        # ===== PLYで保存 =====
        save_path = "/home/araishogo/hotel_ws/src/pointcloud_capture.ply"
        o3d.io.write_point_cloud(save_path, pcd)
        self.get_logger().info(f"📊 点群取得成功: {points.shape[0]} 点 💾 保存: {save_path}")

        # ---- 座標軸（0.1mスケール）追加 ----
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])

        # ===== 可視化 =====
        o3d.visualization.draw_geometries(
            [pcd, axis],
            window_name="Captured PointCloud (with Axis)",
            width=1000,
            height=800,
        )

        # ===== GraspNet推論スクリプト実行 =====
        try:
            self.get_logger().info("🧠 GraspNet推論を開始します...")
            subprocess.run(
                ['python3', '/home/araishogo/hotel_ws/src/grasp_inference/grasp_inference/grasp_inference_node.py'],
                check=True
            )
            self.get_logger().info("✅ GraspNet推論が完了しました。結果を可視化ウィンドウに表示中です。")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"❌ GraspNet推論でエラー発生: {e}")

      
        # ===== GraspNet結果の座標変換 =====
        try:
                result_path = "/home/araishogo/hotel_ws/src/grasp_inference/grasp_result.npy"
                if os.path.exists(result_path):
                        data = np.load(result_path, allow_pickle=True).item()
                        grasp_center_cam = np.array(data['center'])
                        grasp_R_cam = np.array(data['rotation'])
                        self.get_logger().info("📥 GraspNet結果読込完了（camera_camera_camera_link基準）")
                        # ==== カメラ座標系の把持位置と姿勢をログ表示 ====
                        self.get_logger().info(
                                f"\n🎥 【カメラ座標系 把持姿勢】\n"
                                f"  中心[m]: x={grasp_center_cam[0]:.3f}, "
                                f"y={grasp_center_cam[1]:.3f}, z={grasp_center_cam[2]:.3f}\n"
                                f"  回転行列:\n{grasp_R_cam}"
                        )

                        # ✅ TF購読が始まるまで待機（最大5秒）
                        from rclpy.time import Time
                        from rclpy.duration import Duration
                        ok = False
                        for i in range(10):
                                if self.tf_buffer.can_transform("base_link", "camera_camera_camera_link", Time()):
                                        self.get_logger().info(f"✅ TF購読確認成功 ({i+1}回目)")
                                        ok = True
                                        break
                                self.get_logger().warn(f"⌛ TF購読待機中... ({i+1}/10)")
                                time.sleep(0.5)
                        if not ok:
                                self.get_logger().error("❌ TF購読が開始されませんでした。")
                                return

                        # ✅ 最新時刻（0）で取得：最も安定
                        trans = self.tf_buffer.lookup_transform(
                                "base_link",
                                "camera_camera_camera_link",
                                Time(),  # ← ここを最新に
                                timeout=Duration(seconds=2.0)
                        )
                        self.get_logger().info("✅ TF取得成功")

                        # === TFを4x4行列に変換 ===
                        T = tf_transformations.quaternion_matrix([
                                trans.transform.rotation.x,
                                trans.transform.rotation.y,
                                trans.transform.rotation.z,
                                trans.transform.rotation.w,
                        ])
                        T[0, 3] = trans.transform.translation.x
                        T[1, 3] = trans.transform.translation.y
                        T[2, 3] = trans.transform.translation.z

                        # === 回転＋並進を適用 ===
                        R_base = T[:3, :3] @ grasp_R_cam
                        t_base = T[:3, :3] @ grasp_center_cam + T[:3, 3]
                        # === 回転行列 → クォータニオン ===
                        T_base = np.eye(4)
                        T_base[:3, :3] = R_base
                        quat = quaternion_from_matrix(T_base)
                        qx, qy, qz, qw = quat

                        self.get_logger().info(
                                f"\n🧭 【base_link基準 把持姿勢】\n"
                                f"  位置[m]: x={t_base[0]:.3f}, y={t_base[1]:.3f}, z={t_base[2]:.3f}\n"
                                f"  回転行列:\n{R_base}"
                        )

                        self.get_logger().info(
                                f"\n📦 【PoseStamped(base_link) 出力・コピペ用】\n"
                                f"ros2 topic pub --once /target_position geometry_msgs/msg/PoseStamped "
                                f"\"{{header: {{frame_id: 'base_link'}}, pose: {{position: {{x: {t_base[0]:.3f}, y: {t_base[1]:.3f}, z: {t_base[2]:.3f}}}, "
                                f"orientation: {{x: {qx:.3f}, y: {qy:.3f}, z: {qz:.3f}, w: {qw:.3f}}}}}}}\" -1"
                        )

                        # ===== 最初のJointState姿勢を送信 =====
                        pose_start = JointState()
                        pose_start.header.stamp = self.get_clock().now().to_msg()
                        pose_start.header.frame_id = "piper_single"
                        pose_start.name = ['joint1', 'joint2','joint3','joint4','joint5','joint6','joint7']
                        pose_start.position = [0.0,1.2,-0.6,0.0,0.6,0.3,0.035]
                        pose_start.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 30.0]
                        pose_start.effort =   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]

                        self.joint_pub.publish(pose_start)
                        self.get_logger().info("🚀 最初の姿勢 (JointState) を送信しました")
                        time.sleep(4.0)

                        # ===== 中継姿勢（XY同じで Z+0.1） =====
                        safe_pose = PoseStamped()
                        safe_pose.header.frame_id = "base_link"
                        safe_pose.header.stamp = self.get_clock().now().to_msg()

                        safe_pose.pose.position.x = float(t_base[0])
                        safe_pose.pose.position.y = float(t_base[1])
                        safe_pose.pose.position.z = float(t_base[2]) + 0.1

                        safe_pose.pose.orientation.x = float(qx)
                        safe_pose.pose.orientation.y = float(qy)
                        safe_pose.pose.orientation.z = float(qz)
                        safe_pose.pose.orientation.w = float(qw)

                        self.pose_pub.publish(safe_pose)
                        self.get_logger().info("📨 中継姿勢 (Z+0.1) を /target_position に送信しました")
                        time.sleep(5.0)
                        
                        # ===== PoseStamped（本番）を作成して publish =====
                        pose_msg = PoseStamped()
                        pose_msg.header.frame_id = "base_link"
                        pose_msg.header.stamp = self.get_clock().now().to_msg()

                        pose_msg.pose.position.x = float(t_base[0])
                        pose_msg.pose.position.y = float(t_base[1])
                        pose_msg.pose.position.z = float(t_base[2])

                        pose_msg.pose.orientation.x = float(qx)
                        pose_msg.pose.orientation.y = float(qy)
                        pose_msg.pose.orientation.z = float(qz)
                        pose_msg.pose.orientation.w = float(qw)

                        self.pose_pub.publish(pose_msg)
                        self.get_logger().info("📨 本番のPoseStamped を /target_position に送信しました！")

                        # ===== グリッパー閉じる =====
                        time.sleep(2.0)
                        grip_msg = String()
                        grip_msg.data = "close"
                        self.gripper_pub.publish(grip_msg)
                        self.get_logger().info("🖐️ グリッパー閉じる指令を /gripper_cmd に送信しました。")

                        # ===== 最終姿勢へ戻す =====
                        time.sleep(2.0)
                        final_pose = JointState()
                        final_pose.header.stamp = self.get_clock().now().to_msg()
                        final_pose.header.frame_id = "piper_single"
                        final_pose.name = ['joint1','joint2','joint3','joint4','joint5','joint6','joint7']
                        final_pose.position = [0.0,1.4,-0.6,0.0,0.6,-1.3,0.0]
                        final_pose.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0]
                        final_pose.effort   = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]

                        self.joint_pub.publish(final_pose)
                        self.get_logger().info("🔙 最終姿勢（JointState）を送信しました")





                      
                else:
                        self.get_logger().warn("⚠️ GraspNet結果ファイルが見つかりません。")
        except Exception as e:
                self.get_logger().error(f"❌ 座標変換でエラー発生: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = GoalArrivedPointcloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_realsense()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
