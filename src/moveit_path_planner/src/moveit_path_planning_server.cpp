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
#include "custom_interface/srv/movement_request.hpp"
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
      "arm",
      std::shared_ptr<tf2_ros::Buffer>(),
      rclcpp::Duration::from_seconds(5.0)
    );

    // Configure planner parameters
    node_->declare_parameter("planning_time", 1.0);
    node_->declare_parameter("goal_joint_tolerance", 0.01);  // Increased from 0.01
    node_->declare_parameter("goal_position_tolerance", 0.01);  // Increased from 0.01
    node_->declare_parameter("goal_orientation_tolerance", 0.01);  // Increased from 0.01
    node_->declare_parameter("planner_id", "RRTConnectkConfigDefault");

    // Apply parameters
    move_group_->setPlanningTime(node_->get_parameter("planning_time").as_double());
    move_group_->setGoalJointTolerance(node_->get_parameter("goal_joint_tolerance").as_double());
    move_group_->setGoalPositionTolerance(node_->get_parameter("goal_position_tolerance").as_double());
    move_group_->setGoalOrientationTolerance(node_->get_parameter("goal_orientation_tolerance").as_double());

    geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();

    tf2::Quaternion quat;
    tf2::fromMsg(current_pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    RCLCPP_INFO(
      node_->get_logger(),
      "Reached pose:\nPosition - x: %.3f, y: %.3f, z: %.3f\nOrientation - x: %.3f, y: %.3f, z: %.3f, w: %.3f",
      current_pose.pose.position.x,
      current_pose.pose.position.y,
      current_pose.pose.position.z,
      current_pose.pose.orientation.x,
      current_pose.pose.orientation.y,
      current_pose.pose.orientation.z,
      current_pose.pose.orientation.w
    );
    RCLCPP_INFO(
      node_->get_logger(),
      "RPY orientation - roll: %.3f, pitch: %.3f, yaw: %.3f",
      roll, pitch, yaw
    );

    service_ = node_->create_service<custom_interface::srv::MovementRequest>(
      "/moveit_path_plan",
      std::bind(&MoveitPathPlanningServer::handle_request, this, _1, _2)
    );
  }

  moveit_msgs::msg::Constraints create_path_constraints() {
    moveit_msgs::msg::Constraints constraints;

    // End effector orientation constraint (facing downward)
    moveit_msgs::msg::OrientationConstraint orientation_constraint;
    orientation_constraint.header.frame_id = move_group_->getPlanningFrame();
    orientation_constraint.link_name = move_group_->getEndEffectorLink();
    
    // Desired orientation (facing downward: Z-axis pointing down)
    // This depends on your robot's URDF definition
    tf2::Quaternion q;
    q.setRPY(0, M_PI, 0);  // Roll=0, Pitch=π, Yaw=0 (facing downward)
    orientation_constraint.orientation = tf2::toMsg(q);
    
    // Tolerance values (in radians)
    orientation_constraint.absolute_x_axis_tolerance = 1;  // ~5.7 degrees
    orientation_constraint.absolute_y_axis_tolerance = 1;
    orientation_constraint.absolute_z_axis_tolerance = 1;
    orientation_constraint.weight = 0.1;
    
    constraints.orientation_constraints.push_back(orientation_constraint);

    return constraints;
  }

  void handle_request(
    const std::shared_ptr<custom_interface::srv::MovementRequest::Request> request,
    std::shared_ptr<custom_interface::srv::MovementRequest::Response> response)
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
    move_group_->setPathConstraints(create_path_constraints());
    
    // Plan with retries
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;
    int attempts = 0;
    const int max_attempts = 2;
    
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

  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Service<custom_interface::srv::MovementRequest>::SharedPtr service_;
  std::unordered_map<std::string, moveit::planning_interface::MoveGroupInterface::Plan> plan_cache_;
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

//ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{positions: [-0.622, -0.183, 0.055, 0.0, 3.14, 0.0]}"

// should be 

// ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{positions: [0.622, 0.183, 0.255, 0.0, 3.14, 0.0]}"


// birds-eye for camera
// ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{positions: [0.822, 0.183, 0.856, 0.0, 3.14, 0.0]}"

// [INFO] [1749184846.573696882] [camera_client]:   Object 1: X=-0.449m, Y=-0.188m, Z=0.056m
// [INFO] [1749184846.575107469] [camera_client]:   Object 2: X=-0.791m, Y=-0.229m, Z=0.057m
// [INFO] [1749184846.576388168] [camera_client]:   Object 3: X=-0.632m, Y=-0.126m, Z=0.054m
// [INFO] [1749184846.577930020] [camera_client]:   Object 4: X=-0.951m, Y=-0.097m, Z=0.057m