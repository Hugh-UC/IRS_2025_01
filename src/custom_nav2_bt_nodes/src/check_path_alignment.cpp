#include "custom_nav2_bt_nodes/check_path_alignment.hpp"
#include <memory>
#include "nav2_behavior_tree/bt_conversions.hpp"

namespace custom_nav2_bt_nodes
{

CheckPathAlignment::CheckPathAlignment(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  // Create a temporary node for TF access
  node_ = rclcpp::Node::make_shared("path_alignment_checker_temp_node");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Get the threshold from the XML port
  getInput("angle_threshold", angle_threshold_);
}

BT::NodeStatus CheckPathAlignment::tick()
{
  nav_msgs::msg::Path path;
  if (!getInput("path", path)) {
    // Path not on blackboard (shouldn't happen if ComputePathToPose succeeds)
    return BT::NodeStatus::FAILURE;
  }

  // 1. Check Path Sanity
  if (path.poses.size() < 2) {
    // Need at least 2 poses to determine a direction vector
    return BT::NodeStatus::FAILURE;
  }

  // 2. Get Robot Pose
  geometry_msgs::msg::TransformStamped robot_transform;
  std::string error;
  try {
    // Get the transform from map to robot's base_link
    robot_transform = tf_buffer_->lookupTransform(
      path.header.frame_id,
      "virtual_hand_solo/base_link", // Your robot's base frame
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    // Cannot get transform (TF tree broken or timing issue)
    RCLCPP_ERROR(node_->get_logger(), "Could not transform robot pose: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  // 3. Get Robot Heading
  double robot_yaw = tf2::getYaw(robot_transform.transform.rotation);
  
  // 4. Calculate Path Start Vector
  // Use the vector from the first path point to the second path point
  const auto & start_pose = path.poses[0].pose.position;
  const auto & next_pose = path.poses[1].pose.position;

  double dx = next_pose.x - start_pose.x;
  double dy = next_pose.y - start_pose.y;
  
  // Calculate the yaw of the path vector
  double path_yaw = std::atan2(dy, dx);

  // 5. Calculate Angular Difference
  double angle_diff = std::abs(tf2NormalizeAngle(path_yaw - robot_yaw));

  // 6. Check Condition
  if (angle_diff > angle_threshold_) {
    // Robot is misaligned (angle_diff > 120 degrees)
    RCLCPP_WARN(node_->get_logger(), 
      "Alignment failed: Diff %.2f rad (Threshold %.2f rad). Triggering Spin.", 
      angle_diff, angle_threshold_);
    return BT::NodeStatus::FAILURE;
  }

  // Robot is sufficiently aligned
  return BT::NodeStatus::SUCCESS;
}

} // namespace custom_nav2_bt_nodes

#include "pluginlib/class_list_macros.hpp" 

PLUGINLIB_EXPORT_CLASS(
  custom_nav2_bt_nodes::CheckPathAlignment,   // condition node class
  BT::ConditionNode                           // base class (ConditionNode in this case)
)