#!/usr/bin/env python3
import rclpy
import moveit_commander
from geometry_msgs.msg import Pose
import math
import tf_transformations

def main():
    rclpy.init()
    moveit_commander.roscpp_initialize([])

    # "piper_with_gripper_moveit" 内のグループ名
    group = moveit_commander.MoveGroupCommander("piper_arm")

    print("Current pose:")
    print(group.get_current_pose().pose)

    # ===== 目標姿勢の設定 =====
    target_pose = Pose()
    target_pose.position.x = 0.35
    target_pose.position.y = 0.0
    target_pose.position.z = 0.25

    # Z軸下向き姿勢（エンドエフェクタが真下）
    q = tf_transformations.quaternion_from_euler(math.pi, 0.0, 0.0)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]

    group.set_pose_target(target_pose)

    # ===== 計画と実行 =====
    plan = group.plan()
    if not plan:
        print("❌ Planning failed.")
    else:
        print("✅ Plan generated. Executing...")
        group.go(wait=True)

    group.stop()
    group.clear_pose_targets()
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()

