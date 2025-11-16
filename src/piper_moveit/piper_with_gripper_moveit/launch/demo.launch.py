from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os, yaml

# -------------------------------
# YAML ロード用ユーティリティ関数
# -------------------------------
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, "r") as file:
        return yaml.safe_load(file)

# -------------------------------
# LaunchDescription 生成
# -------------------------------
def generate_launch_description():
    # ===== Launch 引数を定義 =====
    can_port_arg = DeclareLaunchArgument(
        "can_port", default_value="can0", description="CAN port for the robot arm"
    )
    auto_enable_arg = DeclareLaunchArgument(
        "auto_enable", default_value="true", description="Enable robot arm automatically"
    )
    gripper_exist_arg = DeclareLaunchArgument(
        "gripper_exist", default_value="true", description="Gripper existence flag"
    )
    gripper_val_mutiple_arg = DeclareLaunchArgument(
        "gripper_val_mutiple", default_value="2", description="Gripper value multiplier"
    )

    # ===== MoveIt 設定読み込み =====
    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit")
        .robot_description(file_path="config/piper.urdf.xacro")
        .robot_description_semantic(file_path="config/piper.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # ====== 標準の demo launch (move_group + rviz) ======
    ld = generate_demo_launch(moveit_config)

    # ✅ 明示的に move_group ノードのパラメータ追加 =====
    kinematics_yaml = load_yaml("piper_with_gripper_moveit", "config/kinematics.yaml")

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            load_yaml("piper_with_gripper_moveit", "config/moveit_planning_pipeline.yaml"),
            load_yaml("piper_with_gripper_moveit", "config/moveit_controllers.yaml"),
        ],
    )

    print("✅ move_group node explicitly configured with KDL solver")
    # ====================================================


    # ===== 自作ノード（my_planner） =====
    my_planner_node = Node(
        package="my_moveit_example",
        executable="my_planner_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,                # ✅ URDFを渡す
            moveit_config.robot_description_semantic,       # ✅ SRDFを渡す
            {"robot_description_kinematics": kinematics_yaml},  # ✅ 逆運動学設定
        ]
    )

    # ===== PIPER 実機制御ノード =====
    piper_ctrl_node = Node(
        package="piper",
        executable="piper_single_ctrl",
        name="piper_ctrl_single_node",
        output="screen",
        parameters=[
            {"can_port": LaunchConfiguration("can_port")},
            {"auto_enable": LaunchConfiguration("auto_enable")},
            {"gripper_val_mutiple": LaunchConfiguration("gripper_val_mutiple")},
            {"gripper_exist": LaunchConfiguration("gripper_exist")},
        ],
        remappings=[
            ("joint_states_single", "/joint_states"),
        ]
    )


    # ===== RViz2 ノード（MoveIt設定付き） =====
    rviz_config_path = os.path.join(
        get_package_share_directory("piper_with_gripper_moveit"),
        "config",
        "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )
    # ===== Robot State Publisher =====
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )


    # ===== LaunchDescriptionにまとめる =====
    return LaunchDescription([
        can_port_arg,
        auto_enable_arg,
        gripper_exist_arg,
        gripper_val_mutiple_arg,
        my_planner_node,
        piper_ctrl_node,
        move_group_node, 
        rviz_node,
        robot_state_pub_node,
       # *ld.entities  # generate_demo_launch が返すノード群を追加
    ])