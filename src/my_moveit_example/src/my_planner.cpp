#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class MyPlannerNode : public rclcpp::Node
{
public:
  MyPlannerNode(moveit::planning_interface::MoveGroupInterface* move_group)
    : Node("my_planner_node"), move_group_(move_group)
  {
    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_position", 10,
        std::bind(&MyPlannerNode::target_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waiting for /target_position ...");
  }

private:
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
                "Received target pose: (%.3f, %.3f, %.3f, q=[%.3f, %.3f, %.3f, %.3f])",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                msg->pose.orientation.x, msg->pose.orientation.y,
                msg->pose.orientation.z, msg->pose.orientation.w);

    // 現在状態を開始に設定
    move_group_->setStartStateToCurrentState();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = msg->header.frame_id.empty() ? "base_link" : msg->header.frame_id;
    target_pose.header.stamp = this->now();

    // --- 入力Poseをそのまま使用 ---
    target_pose.pose = msg->pose;

    // --- 許容誤差（やや厳しめ） ---
    move_group_->setGoalPositionTolerance(0.05);
    move_group_->setGoalOrientationTolerance(0.3);

    move_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    auto plan_result = move_group_->plan(my_plan);

    if (plan_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Planning succeeded, executing...");
      auto exec_result = move_group_->execute(my_plan);

      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
        RCLCPP_INFO(this->get_logger(), "Execution succeeded!");
      else
        RCLCPP_ERROR(this->get_logger(), "Execution failed! Code: %d", exec_result.val);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Planning failed! Code: %d", plan_result.val);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  moveit::planning_interface::MoveGroupInterface* move_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto dummy_node = rclcpp::Node::make_shared("dummy_node_for_move_group");
  moveit::planning_interface::MoveGroupInterface move_group(dummy_node, "arm");

  auto node = std::make_shared<MyPlannerNode>(&move_group);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
