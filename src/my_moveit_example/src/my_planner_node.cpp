#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/string.hpp>

class MyPlannerNode : public rclcpp::Node
{
public:
    MyPlannerNode(moveit::planning_interface::MoveGroupInterface* arm_group,
                  moveit::planning_interface::MoveGroupInterface* gripper_group)
        : Node("my_planner_node"), arm_group_(arm_group), gripper_group_(gripper_group)
    {
        // アーム用Pose購読
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_position",
            10,
            std::bind(&MyPlannerNode::target_callback, this, std::placeholders::_1)
        );

        // グリッパ用コマンド購読
        sub_gripper_ = this->create_subscription<std_msgs::msg::String>(
            "/gripper_cmd",
            10,
            std::bind(&MyPlannerNode::gripper_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "✅ Waiting for /target_position and /gripper_cmd ...");
    }

private:
    // ======== アーム制御 ========
    void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Received target pose: (%.3f, %.3f, %.3f, q=[%.3f, %.3f, %.3f, %.3f])",
                    msg->pose.position.x,
                    msg->pose.position.y,
                    msg->pose.position.z,
                    msg->pose.orientation.x,
                    msg->pose.orientation.y,
                    msg->pose.orientation.z,
                    msg->pose.orientation.w);

        arm_group_->setStartStateToCurrentState();
        arm_group_->setPlanningTime(10.0);
        arm_group_->setGoalPositionTolerance(0.01);
        arm_group_->setGoalOrientationTolerance(0.03);
        arm_group_->setMaxVelocityScalingFactor(0.5);
        arm_group_->setMaxAccelerationScalingFactor(0.5);

        geometry_msgs::msg::PoseStamped target_pose = *msg;
        if (target_pose.header.frame_id.empty())
            target_pose.header.frame_id = "base_link";

        if (!arm_group_->setPoseTarget(target_pose))
        {
            RCLCPP_ERROR(this->get_logger(), "⚠️ Invalid pose target (probably unreachable)");
            return;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = arm_group_->plan(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "✅ Planning succeeded, executing...");
            auto exec_result = arm_group_->execute(plan);
            if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
                RCLCPP_INFO(this->get_logger(), "✅ Execution succeeded!");
            else
                RCLCPP_ERROR(this->get_logger(), "❌ Execution failed! Code: %d", exec_result.val);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Planning failed! Code: %d", result.val);
        }
    }

    // ======== グリッパ開閉 ========
    void gripper_callback(const std_msgs::msg::String::SharedPtr msg)
    {   
        gripper_group_->setStartStateToCurrentState();
        if (msg->data == "open")
        {
            RCLCPP_INFO(this->get_logger(), "🖐️ Opening gripper...");
            gripper_group_->setNamedTarget("open");
            gripper_group_->move();
        }
        else if (msg->data == "close")
        {
            RCLCPP_INFO(this->get_logger(), "✊ Closing gripper...");
            gripper_group_->setNamedTarget("close");
            gripper_group_->move();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ Unknown gripper command: '%s'", msg->data.c_str());
        }
    }

    // ======== メンバ変数 ========
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_gripper_;
    moveit::planning_interface::MoveGroupInterface* arm_group_;
    moveit::planning_interface::MoveGroupInterface* gripper_group_;
};

// ======== main() ========
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto dummy_node = rclcpp::Node::make_shared("dummy_node_for_move_group");

    // 🌟 MoveGroupの初期化
    moveit::planning_interface::MoveGroupInterface arm_group(dummy_node, "arm");
    moveit::planning_interface::MoveGroupInterface gripper_group(dummy_node, "gripper_group");

    // 🌟 現在の状態をMoveItに反映
    arm_group.setStartStateToCurrentState();
    gripper_group.setStartStateToCurrentState();


    // 🌟 ノード生成（両方渡す）
    auto node = std::make_shared<MyPlannerNode>(&arm_group, &gripper_group);

    // 🌟 起動時にグリッパを開く
    //gripper_group.setNamedTarget("open");
    //gripper_group.move();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
