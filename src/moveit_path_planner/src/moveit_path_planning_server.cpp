#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "moveit_msgs/msg/orientation_constraint.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tm_msgs/srv/movement_request.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h> 

class MoveitPathPlanningServer {
public:
  MoveitPathPlanningServer(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  {
    using namespace std::placeholders;

    RCLCPP_INFO(node_->get_logger(), "Starting MoveIt Path Planning Server...");

    // Initialize MoveGroupInterface with proper parameters
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_, 
      "ur_manipulator",
      std::shared_ptr<tf2_ros::Buffer>(),
      rclcpp::Duration::from_seconds(5.0)
    );

    // Configure planner parameters
    node_->declare_parameter("planning_time", 10.0);
    node_->declare_parameter("goal_joint_tolerance", 0.01);
    node_->declare_parameter("goal_position_tolerance", 0.01);
    node_->declare_parameter("goal_orientation_tolerance", 0.01);
    node_->declare_parameter("planner_id", "RRTConnectkConfigDefault");

    // Apply parameters
    move_group_->setPlanningTime(node_->get_parameter("planning_time").as_double());
    move_group_->setGoalJointTolerance(node_->get_parameter("goal_joint_tolerance").as_double());
    move_group_->setGoalPositionTolerance(node_->get_parameter("goal_position_tolerance").as_double());
    move_group_->setGoalOrientationTolerance(node_->get_parameter("goal_orientation_tolerance").as_double());
    move_group_->setPlannerId(node_->get_parameter("planner_id").as_string());

    // Set velocity and acceleration scaling
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);

    service_ = node_->create_service<tm_msgs::srv::MovementRequest>(
      "/moveit_path_plan",
      std::bind(&MoveitPathPlanningServer::handle_request, this, _1, _2)
    );
  }

  moveit_msgs::msg::Constraints create_path_constraints() {
    moveit_msgs::msg::Constraints constraints;
    
    // // Joint constraints for joint4 (-45 to +45 degrees)
    // moveit_msgs::msg::JointConstraint joint4_constraint;
    // joint4_constraint.joint_name = "joint_4";
    // joint4_constraint.position = 0.0;  // Center of range
    // joint4_constraint.tolerance_above = 0.785;  // 45 degrees in radians
    // joint4_constraint.tolerance_below = 0.785;
    // joint4_constraint.weight = 1.0;
    // constraints.joint_constraints.push_back(joint4_constraint);

    // // Joint constraints for joint5 (-45 to +135 degrees)
    // moveit_msgs::msg::JointConstraint joint5_constraint;
    // joint5_constraint.joint_name = "joint_5";
    // joint5_constraint.position = 1.5708;  // Center of range (90 degrees)
    // joint5_constraint.tolerance_below = 0.785;  // 45 degrees in radians
    // joint5_constraint.tolerance_above = 0.785;  // 45 degrees in radians
    // joint5_constraint.weight = 1.0;
    // constraints.joint_constraints.push_back(joint5_constraint);

    // // End effector orientation constraint (facing downward)
    // moveit_msgs::msg::OrientationConstraint orientation_constraint;
    // orientation_constraint.header.frame_id = move_group_->getPlanningFrame();
    // orientation_constraint.link_name = move_group_->getEndEffectorLink();
    
    // // Desired orientation (facing downward: Z-axis pointing down)
    // // This depends on your robot's URDF definition
    // tf2::Quaternion q;
    // q.setRPY(0, M_PI, 0);  // Roll=0, Pitch=π, Yaw=0 (facing downward)
    // orientation_constraint.orientation = tf2::toMsg(q);
    
    // // Tolerance values (in radians)
    // orientation_constraint.absolute_x_axis_tolerance = 0.5;  // ~5.7 degrees
    // orientation_constraint.absolute_y_axis_tolerance = 0.5;
    // orientation_constraint.absolute_z_axis_tolerance = 0.5;
    // orientation_constraint.weight = 1.0;
    
    // constraints.orientation_constraints.push_back(orientation_constraint);

    moveit_msgs::msg::Constraints path_constraints;
    moveit_msgs::msg::JointConstraint joint6_constraint;

    joint6_constraint.joint_name = "joint_6";
    joint6_constraint.position = 0.0;  // Centered at 0 rad, adjust if needed
    joint6_constraint.tolerance_above = M_PI;     // Allow 180 deg in positive
    joint6_constraint.tolerance_below = M_PI;     // Allow 180 deg in negative
    joint6_constraint.weight = 1.0;

    path_constraints.joint_constraints.push_back(joint6_constraint);

    return constraints;
  }

  void handle_request(
    const std::shared_ptr<tm_msgs::srv::MovementRequest::Request> request,
    std::shared_ptr<tm_msgs::srv::MovementRequest::Response> response)
  {
    RCLCPP_INFO(node_->get_logger(), "Received MoveIt path planning request.");

    const std::vector<double>& positions = request->positions;

    if (positions.size() != 6) {
      RCLCPP_ERROR(node_->get_logger(), "Expected 6 pose elements (x, y, z, roll, pitch, yaw), got %zu", positions.size());
      response->success = false;
      return;
    }
    
    // Extract Cartesian position and orientation
    double x = positions[0];
    double y = positions[1];
    double z = positions[2];
    double roll = positions[3];
    double pitch = positions[4];
    double yaw = positions[5];
    
    // Convert RPY to Quaternion
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();  // Optional but recommended
    
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();
    
    // Set Cartesian pose target
    move_group_->setPoseTarget(target_pose);
    // move_group_->setPathConstraints(create_path_constraints());
    
    // Plan with retries
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;
    int attempts = 0;
    const int max_attempts = 5;
    
    while (!success && attempts < max_attempts) {
      success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      attempts++;
      if (!success) {
        RCLCPP_WARN(node_->get_logger(), "Planning attempt %d failed, retrying...", attempts);
        move_group_->setPlanningTime(move_group_->getPlanningTime() + 2.0);
      }
    }
    
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "Plan successful after %d attempts. Executing...", attempts);
      move_group_->execute(plan);
      response->success = true;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed after %d attempts.", max_attempts);
      response->success = false;
    }
    // const std::vector<double>& positions = request->positions;

    // if (positions.size() != 6) {
    //   RCLCPP_ERROR(node_->get_logger(), "Expected 6 joint positions, got %zu", positions.size());
    //   response->success = false;
    //   return;
    // }

    // // Set path constraints
    // // move_group_->setPathConstraints(create_path_constraints());

    // // Set joint target
    // move_group_->setJointValueTarget(positions);

    // // Plan with retries
    // moveit::planning_interface::MoveGroupInterface::Plan plan;
    // bool success = false;
    // int attempts = 0;
    // const int max_attempts = 5;

    // while (!success && attempts < max_attempts) {
    //   success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    //   attempts++;
    //   if (!success) {
    //     RCLCPP_WARN(node_->get_logger(), "Planning attempt %d failed, retrying...", attempts);
    //     move_group_->setPlanningTime(move_group_->getPlanningTime() + 2.0);
    //   }
    // }

    // // Clear constraints after planning
    // move_group_->clearPathConstraints();

    // if (success) {
    //   RCLCPP_INFO(node_->get_logger(), 
    //              "Plan successful after %d attempts. Executing...", 
    //              attempts);
    //   move_group_->execute(plan);
    //   response->success = true;
    // } else {
    //   RCLCPP_ERROR(node_->get_logger(), 
    //               "Planning failed after %d attempts.", 
    //               max_attempts);
    //   response->success = false;
    // }
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Service<tm_msgs::srv::MovementRequest>::SharedPtr service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("moveit_path_planning_server");
  MoveitPathPlanningServer server(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// [INFO] [1749099580.306942371] [camera_server]: Transformed coordinates (base frame): X: -0.622, Y: -0.183, Z: 0.055
// [INFO] [1749099580.313162098] [camera_server]: Transformed coordinates (base frame): X: -0.700, Y: -0.280, Z: 0.061
// [INFO] [1749099580.316878851] [camera_server]: Transformed coordinates (base frame): X: -0.542, Y: -0.372, Z: 0.057
// [INFO] [1749099580.319195569] [camera_server]: Transformed coordinates (base frame): X: -0.472, Y: -0.288, Z: 0.062

//ros2 service call /moveit_path_plan tm_msgs/srv/MovementRequest "{positions: [-0.622, -0.183, 0.055, 0.0, 3.14, 0.0]}"
