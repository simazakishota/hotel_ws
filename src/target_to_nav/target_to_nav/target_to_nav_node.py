#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from tf_transformations import quaternion_from_euler
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
from rclpy.duration import Duration


class TargetToNavNode(Node):
    def __init__(self):
        super().__init__('target_to_nav_node')

        # --- TF Buffer ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Nav2 Navigator ---
        self.navigator = BasicNavigator()
        self.get_logger().info('⏳ Waiting for Nav2 server...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('✅ Nav2 server active')

        # --- AMCL Pose ---
        self.current_pose = None
        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            amcl_qos
        )

        # --- Publishers ---
        self.goal_done_pub = self.create_publisher(Bool, '/nav_goal_done', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_viz', 10)

        # --- Target subscriber (base_link座標で受信) ---
        self.create_subscription(
            PointStamped,
            '/target_in_camera',
            self.target_callback,
            10
        )

        # --- パラメータ ---
        self.arrival_threshold = 0.15  # 距離[m]
        self.timeout_sec = 120.0       # タイムアウト[s]
        self.stop_confirm_time = 2.0   # 停止確認時間[s]

        self.get_logger().info('✅ target_to_nav_node started. Waiting for /target_in_camera ...')

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    def target_callback(self, msg: PointStamped):
        self.get_logger().info(
            f"📡 Target in base_link frame: (x={msg.point.x:.2f}, y={msg.point.y:.2f})"
        )

        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0

        try:
            point_map = self.tf_buffer.transform(
                msg, 'map', timeout=Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f"❌ TF transform failed: {e}")
            return

        self.get_logger().info(
            f"🌍 Target in map frame: (x={point_map.point.x:.2f}, y={point_map.point.y:.2f})"
        )
        self.send_goal_to_nav(point_map)

    def send_goal_to_nav(self, point_map: PointStamped):
        if self.current_pose is None:
            self.get_logger().error('❌ /amcl_pose が未取得')
            return

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        target_x = point_map.point.x
        target_y = point_map.point.y

        offset = 0.17
        dx, dy = target_x - robot_x, target_y - robot_y
        dist = math.hypot(dx, dy)
        if dist < 0.1:
            self.get_logger().warn('⚠️ ターゲットが近すぎる')
            return

        ux, uy = dx / dist, dy / dist
        goal_x, goal_y = target_x - offset, target_y 
        yaw = 0.0
        q = quaternion_from_euler(0, 0, yaw)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        self.goal_pub.publish(goal_pose)
        self.get_logger().info(f"🚀 Sending Nav2 goal: ({goal_x:.2f}, {goal_y:.2f}), yaw={yaw:.2f}")

        self.navigator.goToPose(goal_pose)
        start_time = self.get_clock().now()
        goal_reached = False
        last_pose = None
        still_time = 0.0

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                remaining = feedback.distance_remaining
                self.get_logger().info(f"🚗 Remaining distance: {remaining:.2f} m")

                # 残距離が閾値以下なら「静止確認」フェーズに入る
                if remaining < self.arrival_threshold and self.current_pose:
                    if last_pose:
                        dx = self.current_pose.position.x - last_pose[0]
                        dy = self.current_pose.position.y - last_pose[1]
                        move_dist = math.hypot(dx, dy)
                        if move_dist < 0.01:  # ほぼ停止
                            still_time += 0.1
                        else:
                            still_time = 0.0
                    last_pose = (self.current_pose.position.x, self.current_pose.position.y)
                    if still_time >= self.stop_confirm_time:
                        goal_reached = True
                        break

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > self.timeout_sec:
                self.get_logger().error('⏱️ Navigation timeout!')
                self.navigator.cancelTask()
                break

            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()
        if goal_reached or result == TaskResult.SUCCEEDED:
            self.get_logger().info('✅ Navigation complete & robot stopped.')
            self.goal_done_pub.publish(Bool(data=True))
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('⚠️ Navigation canceled.')
        else:
            self.get_logger().error('❌ Navigation failed.')


def main(args=None):
    rclpy.init(args=args)
    node = TargetToNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
