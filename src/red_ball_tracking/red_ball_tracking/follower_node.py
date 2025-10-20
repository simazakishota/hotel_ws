#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Int8
import numpy as np
from std_msgs.msg import Float32

class RedBallFollower(Node):

    def __init__(self):
        super().__init__('red_ball_follower')

        # --- パラメータ ---
        self.kp_ang_x = 0 #3   # X方向の角速度ゲイン
        self.kp_ang_y = 0 #3   # Y方向の角速度ゲイン

        self.kp_z   = 5.0    # 比例ゲイン 
        self.kd_z   = 1.1    # 微分ゲイン
        self.ki_z   = 1.0    # 積分ゲイン

        # --- フィードフォワード項（振り子モデルに基づく） ---
        self.kff = 0.2  # フィードフォワードゲイン
        self.g = 9.81  # [m/s^2]
        self.L = 1.65   # 振り子の長さ [m] ← 実際の実験値に合わせて変更してください
        self.target_ref = 0.92
        # 固有角周波数 ω
        self.omega = np.sqrt(self.g / self.L)

        self.z_threshold = 0.6

        self.integral_error = 0.0
        # --- D制御用 ---
        self.deriv_filtered = 0.0
        self.prev_error_d = 0.0
        self.prev_time = self.get_clock().now()

        # 状態管理
        self.servo_status = 0
        self.following_active = False  # True=追従中, False=待機
        self.last_status = None

        # Standby姿勢
        self.standby_pose = [0.0, 1.1, -1.5, 0.0, 1.0, 0.0]

        # target監視
        self.last_target_time = self.get_clock().now()
        self.target_timeout = 2.0

        # Publisher
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.traj_pub  = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.error_pub = self.create_publisher(Float32, '/redball/error_d', 10)
        self.create_subscription(PoseStamped, '/target_position', self.position_callback, 10)
        # Subscriber
        self.create_subscription(PoseStamped, '/target_camera', self.target_callback, 10)
        self.create_subscription(Int8, '/servo_node/status', self.status_callback, 10)

        # タイマーでターゲット途絶をチェック
        self.create_timer(0.5, self.check_target_timeout)

        self.get_logger().info("RedBallFollower ready (競合回避版)")

        self.create_timer(1.0, self.init_standby_once)
        self.init_sent = False
        self.z_filtered = None
        self.lpf_alpha_z = 0.4 
        self.latest_x = None

    def init_standby_once(self):
        """起動直後に一度だけスタンバイ姿勢を送る"""
        if not hasattr(self, "init_sent") or not self.init_sent:
            self.publish_standby_pose()
            self.get_logger().info("Initial standby trajectory sent")
            self.init_sent = True

    def position_callback(self, msg: PoseStamped):
        """ /target_position (base_link座標系) から x 座標をローパス処理して保存 """
        raw_x = msg.pose.position.x

        # === ローパスフィルタ適用 ===
        if not hasattr(self, "x_filtered") or self.x_filtered is None:
            self.x_filtered = raw_x
        else:
            alpha_x = 0.3  # 0.0～1.0 (小さいほど滑らか)
            self.x_filtered = (1 - alpha_x) * self.x_filtered + alpha_x * raw_x

        # フィルタ後の値を使用
        self.latest_x = self.x_filtered

    # --- Standby姿勢を送信 ---
    def publish_standby_pose(self):
        traj = JointTrajectory()
        traj.joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']

        point = JointTrajectoryPoint()
        point.positions = self.standby_pose
        point.time_from_start.sec = 5
        traj.points.append(point)

        self.traj_pub.publish(traj)
        self.get_logger().info("Standby trajectory sent")

    # --- Servo status の監視 ---
    def status_callback(self, msg: Int8):
        self.servo_status = msg.data

        # 状態=2 → 強制待機モード
        if self.servo_status == 2:
            self.following_active = False
            self.publish_standby_pose()

        # 状態が2以外 → 再び追従可能
       # if self.servo_status != 2:
           # self.get_logger().info("Servo resumed, waiting for target...")

        self.last_status = self.servo_status

    # --- ターゲット途絶チェック ---
    def check_target_timeout(self):
        elapsed = (self.get_clock().now() - self.last_target_time).nanoseconds * 1e-9
        if elapsed > self.target_timeout and self.following_active:
            # 追従が止まってから待機へ
            self.following_active = False
            self.publish_standby_pose()
            self.get_logger().warn("Target lost → switched to standby")

    # --- ターゲット座標を受け取ったとき ---
    def target_callback(self, msg: PoseStamped):
        self.last_target_time = self.get_clock().now()

        if self.servo_status == 2:
            # status=2なら追従禁止
            return

        # ターゲット復帰 → 追従開始
        if not self.following_active:
            self.following_active = True
            self.get_logger().info("Target reacquired → resume following")

        if not self.following_active:
            return

        dx, dy, dz = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z

        # === ローパスフィルタ適用 ===
        if self.z_filtered is None:
            self.z_filtered = dz  # 初回はそのまま
        else:
            self.z_filtered = (1 - self.lpf_alpha_z) * self.z_filtered + self.lpf_alpha_z * dz

        # dz をフィルタ後の値で置き換え
        dz = self.z_filtered

        # --- 誤差を角度に変換 ---
        theta_x = -np.arctan2(dy, np.sqrt(dz**2))
        theta_y =  np.arctan2(dx, np.sqrt(dz**2))
        theta_z = -np.arctan2(dx, dy)

        ang_x = self.kp_ang_x * theta_x
        ang_y = self.kp_ang_y * theta_y
        ang_z = 0.0 * theta_z

        dist = np.sqrt(dx**2 + dy**2 + dz**2)
        error_d = dist - self.z_threshold

        # 時間差を計算
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0:
            dt = 1e-6  # 0割り防止

        # 誤差の変化率（微分項）
        alpha = 0.3  # 0～1 (小さいほど滑らか)
        derivative = (error_d - self.prev_error_d) / dt
        self.deriv_filtered = (1 - alpha) * self.deriv_filtered + alpha * derivative

       # 積分項の更新
        self.integral_error += error_d * dt

       # 積分項の発散防止（アンチワインドアップ）
        max_integral = 1.0
        self.integral_error = max(min(self.integral_error, max_integral), -max_integral)

        feedforward_z = 0.0
        displacement_x = 0.0 
        if self.latest_x is not None:
            displacement_x = self.latest_x - self.target_ref
            feedforward_z = - (self.omega**2) * displacement_x

        # PID制御
        vel_z = (
            self.kp_z * error_d +
            self.ki_z * self.integral_error +
            self.kd_z * self.deriv_filtered +
            self.kff * feedforward_z
        )

        # 値を更新
        self.prev_error_d = error_d
        self.prev_time = now

        # --- Twist生成 ---
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "camera_camera_camera_link"
        twist.twist.linear.z = float(vel_z)
        twist.twist.angular.x = float(ang_x)
        twist.twist.angular.y = float(ang_y)
        twist.twist.angular.z = float(ang_z)

        # --- Publish ---
        self.twist_pub.publish(twist)

        self.get_logger().info(
            f"[FOLLOW] Pos=({dx:.2f},{dy:.2f},{dz:.2f}), "
            f"Ang=({ang_x:.2f},{ang_y:.2f},{ang_z:.2f}), "
            f" Dist={dist:.2f}, Base_dist={displacement_x:.2f}, ErrorD={error_d:.3f}, "
            f"VelZ={vel_z:.2f}, Deriv={self.deriv_filtered:.2f}"
        )

        # --- 誤差をPublish ---
        err_msg = Float32()
        err_msg.data = float(error_d)
        self.error_pub.publish(err_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RedBallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
