#include <string>
#include <cmath>
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav2_behavior_tree/bt_action_node.hpp" // Use the standard BT header
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/path.hpp"

namespace custom_nav2_bt_nodes
{
/**
 * @brief Condition to check the angular misalignment between the robot and the start of the path.
 */
class CheckPathAlignment : public BT::ConditionNode
{
public:
  CheckPathAlignment(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);
  
  CheckPathAlignment() : BT::ConditionNode("EmptyName", {}) {}
  
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList{
      BT::InputPort<nav_msgs::msg::Path>("path", "Global path to check"),
      BT::InputPort<double>("angle_threshold", 2.094, "Max allowed angle (radians) between robot and path start")
    };
  }

private:
  double angle_threshold_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Node::SharedPtr node_;
};

} // namespace custom_nav2_bt_nodes