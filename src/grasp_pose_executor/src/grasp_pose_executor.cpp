#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp/parameter_client.hpp>

class GraspPoseExecutor : public rclcpp::Node
{
public:
  GraspPoseExecutor(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("grasp_pose_executor", options)
  {
    // --- MoveItのパラメータを /move_group からコピー ---
    auto client = std::make_shared<rclcpp::SyncParametersClient>(this, "/move_group");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "⏳ Waiting for /move_group parameter service...");
    }

    copy_param_with_wait(client, "robot_description");
    copy_param_with_wait(client, "robot_description_semantic");
    copy_param_with_wait(client, "robot_description_kinematics");

    // --- grasp_pose購読 ---
    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/grasp_pose", 10,
        std::bind(&GraspPoseExecutor::poseCallback, this, std::placeholders::_1));

    // --- Piper用 JointTrajectory publisher ---
    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/arm_controller/joint_trajectory", 10);

    // --- MoveGroupInterface 初期化を少し遅延 ---
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&GraspPoseExecutor::initMoveGroup, this));
  }

private:
  void copy_param_with_wait(
      const std::shared_ptr<rclcpp::SyncParametersClient> &client,
      const std::string &param_name)
  {
    std::string value;
    for (int i = 0; i < 10; ++i)
    {
      if (client->has_parameter(param_name))
      {
        value = client->get_parameter<std::string>(param_name);
        if (!value.empty()) break;
      }
      RCLCPP_WARN(this->get_logger(), "⏳ Waiting for %s ...", param_name.c_str());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (!value.empty())
    {
      this->declare_parameter(param_name, value);
      this->set_parameter(rclcpp::Parameter(param_name, value));
      RCLCPP_INFO(this->get_logger(), "✅ Copied %s from /move_group", param_name.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to fetch %s from /move_group", param_name.c_str());
    }
  }

  void initMoveGroup()
  {
    if (!move_group_)
    {
      RCLCPP_INFO(this->get_logger(), "🦾 Initializing MoveGroupInterface...");
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          this->shared_from_this(), "arm");

      move_group_->setPlanningTime(5.0);
      move_group_->setMaxVelocityScalingFactor(0.2);
      move_group_->setMaxAccelerationScalingFactor(0.2);
      move_group_->setNumPlanningAttempts(5);

      RCLCPP_INFO(this->get_logger(), "⚙️ Planning initial 'zero' pose...");
      move_group_->setNamedTarget("zero");
      moveit::planning_interface::MoveGroupInterface::Plan init_plan;
      if (move_group_->plan(init_plan) == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "✅ Initial plan success. Publishing trajectory...");
        traj_pub_->publish(init_plan.trajectory_.joint_trajectory);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "❌ Failed to plan zero pose.");
      }
    }
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!move_group_)
    {
      RCLCPP_WARN(this->get_logger(), "MoveGroupInterface not ready yet!");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "📥 Received grasp pose: (%.3f, %.3f, %.3f)",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    move_group_->setPoseTarget(msg->pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "✅ Plan found. Publishing to Piper...");
      traj_pub_->publish(plan.trajectory_.joint_trajectory);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "❌ Planning failed.");
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GraspPoseExecutor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
