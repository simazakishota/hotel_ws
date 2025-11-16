# hotel_ws

ROS 2 (Humble) workspace for hotel service robot development.  
Includes control, perception, and manipulation packages for the Piper arm and Scout Mini base.

## 📦 Main Packages
- **piper_with_gripper_moveit** – MoveIt 2 setup for the Piper arm
- **object_recognition** – YOLOv8-based trash-can detection
- **red_ball_tracking / sphere_tracker** – Color-based and 3D-tracking nodes
- **scout_ros2 / ugv_sdk** – Base-level control
- **grasp_pose_executor / moveit_pose_control** – Arm trajectory control and execution

## ⚙️ Build
```bash
cd ~/hotel_ws
colcon build --symlink-install
source install/setup.bash
