#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using String = example_interfaces::msg::String;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;
using namespace std::placeholders;

class CommanderNode
{
  public:
    CommanderNode(std::shared_ptr<rclcpp::Node> node)
    {
      node_ = node;
      arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
      arm_->setMaxVelocityScalingFactor(1.0);
      arm_->setMaxAccelerationScalingFactor(1.0);

      gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

      open_gripper_sub_ = node->create_subscription<Bool>("open_gripper", 10, std::bind(&CommanderNode::openGripperCallback, this, _1));
      named_target_sub_ = node->create_subscription<String>("named_target", 10, std::bind(&CommanderNode::namedTargetCallback, this, _1));
      joint_target_sub_ = node->create_subscription<Float64MultiArray>("joint_target", 10, std::bind(&CommanderNode::jointTargetCallback, this, _1));
    }

    void goToNamedTarget(const std::string &name)
    {
      arm_->setStartStateToCurrentState();
      arm_->setNamedTarget(name);
      planAndExecute(arm_);
    }

    void goToJointTarget(const std::vector<double> &joints)
    {
      arm_->setStartStateToCurrentState();
      arm_->setJointValueTarget(joints);
      planAndExecute(arm_);
    }

    void goToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path=false)
    {
      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);
      q = q.normalize();

      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.header.frame_id = "base_link";
      target_pose.pose.position.x = x;
      target_pose.pose.position.y = y;
      target_pose.pose.position.z = z;
      target_pose.pose.orientation.x = q.getX();
      target_pose.pose.orientation.y = q.getY();
      target_pose.pose.orientation.z = q.getZ();
      target_pose.pose.orientation.w = q.getW();

      arm_->setStartStateToCurrentState();

      if (!cartesian_path){
        arm_->setPoseTarget(target_pose);
        planAndExecute(arm_);
      }else{
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose.pose);
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);

        if (fraction == 1){
          arm_->execute(trajectory);
        }
      }
    }

    void openGripper()
    {
      gripper_->setStartStateToCurrentState();
      gripper_->setNamedTarget("gripper_open");
      planAndExecute(gripper_);
    }

    void closeGripper()
    {
      gripper_->setStartStateToCurrentState();
      gripper_->setNamedTarget("gripper_close");
      planAndExecute(gripper_);
    }

  private:

    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
      MoveGroupInterface::Plan plan;
      bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (success){
        interface->execute(plan);
      }
    }
    
    void namedTargetCallback(const String &msg)
    {
      goToNamedTarget(msg.data);
    }

    void jointTargetCallback(const Float64MultiArray &msg)
    {
      goToJointTarget(msg.data);
    }

    void openGripperCallback(const Bool &msg)
    {
      if(msg.data){
        openGripper();
      }else{
        closeGripper();
      }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<String>::SharedPtr named_target_sub_;
    rclcpp::Subscription<Float64MultiArray>::SharedPtr joint_target_sub_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("CommanderNode");
    auto commande = CommanderNode(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
