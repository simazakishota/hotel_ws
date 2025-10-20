from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os, yaml
import xacro
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils.launches import generate_demo_launch

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():

    # ===== Launch Arguments =====
    can_port_arg = DeclareLaunchArgument(
        "can_port", default_value="can0",
        description="CAN interface port (e.g., can0)"
    )
    auto_enable_arg = DeclareLaunchArgument(
        "auto_enable", default_value="true",
        description="Enable robot automatically on startup"
    )
    gripper_val_mutiple_arg = DeclareLaunchArgument(
        "gripper_val_mutiple", default_value="1.0",
        description="Multiplier for gripper values"
    )
    gripper_exist_arg = DeclareLaunchArgument(
        "gripper_exist", default_value="true",
        description="Whether gripper exists"
    )

    # ===== MoveIt Config =====
    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit")
        .robot_description(file_path="config/piper.urdf.xacro")
        .robot_description_semantic(file_path="config/piper.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    servo_yaml = load_yaml("piper_with_gripper_moveit", "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    rviz_config_file = os.path.join(
        get_package_share_directory("piper_with_gripper_moveit"),
        "config",
        "moveit.rviz",
    )
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    # ===== 引数定義 =====
    gripper_val_mutiple_arg = DeclareLaunchArgument(
        "gripper_val_mutiple",
        default_value="2",   # 文字列だが、この後 int に変換する
        description="Gripper value multiplier"
    )

    # ===== Piper 実機制御ノード =====
    piper_ctrl_node = Node(
        package="piper",
        executable="piper_single_ctrl",
        name="piper_ctrl_single_node",
        output="screen",
        parameters=[{
            "can_port": LaunchConfiguration("can_port"),
            "auto_enable": LaunchConfiguration("auto_enable"),
            "gripper_exist": LaunchConfiguration("gripper_exist"),
        # int として渡す
            "gripper_val_mutiple": ParameterValue(LaunchConfiguration("gripper_val_mutiple"), value_type=int),
        }],
        remappings=[
            ("joint_ctrl_single", "/joint_states"),
        ],
    )

    ld = generate_demo_launch(moveit_config)

    return LaunchDescription([
        can_port_arg,
        auto_enable_arg,
        gripper_val_mutiple_arg,
        gripper_exist_arg,
        servo_node,
        piper_ctrl_node,
        *ld.entities 
    ])
