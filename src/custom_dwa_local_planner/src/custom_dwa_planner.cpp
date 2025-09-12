#include "/custom/dwa_local_planner/custom_dwa_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace custom_dwa_local_planner
{

void CustomDWAPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  // Initialize your DWA parameters here from the parameter server
  // For example:
  // node->get_parameter(plugin_name_ + ".max_speed_x", max_speed_x_);
}

void CustomDWAPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up plugin %s", plugin_name_.c_str());
}

void CustomDWAPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s", plugin_name_.c_str());
}

void CustomDWAPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s", plugin_name_.c_str());
}

geometry_msgs::msg::TwistStamped CustomDWAPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & /*pose*/,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;

  // --- Your DWA Logic Goes Here ---
  // 1. Discretize the robot's velocity space (linear and angular velocities).
  // 2. For each sampled velocity, forward simulate the robot's motion for a short time.
  // 3. Evaluate each simulated trajectory based on a cost function (e.g., proximity to obstacles, progress towards the goal, alignment with the global path).
  // 4. Select the best trajectory and its corresponding velocity.
  // 5. Populate cmd_vel with the chosen linear and angular velocities.

  // For now, let's just return a zero velocity
  cmd_vel.header.stamp = node_.lock()->now();
  cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;

  return cmd_vel;
}

void CustomDWAPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void CustomDWAPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  // Implement your speed limit logic here
}

}  // namespace custom_dwa_local_planner

PLUGINLIB_EXPORT_CLASS(custom_dwa_local_planner::CustomDWAPlanner, nav2_core::Controller)
