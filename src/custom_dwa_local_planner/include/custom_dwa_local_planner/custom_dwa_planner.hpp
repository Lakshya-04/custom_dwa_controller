#ifndef CUSTOM_DWA_LOCAL_PLANNER_HPP_
#define CUSTOM_DWA_LOCAL_PLANNER_HPP_

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace custom_dwa_local_planner
{

class CustomDWAPlanner : public nav2_core::Controller
{
public:
  CustomDWAPlanner() = default;
  ~CustomDWAPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
  rclcpp::Logger logger_{rclcpp::get_logger("CustomDWAPlanner")};
  // Add your DWA-specific member variables here
};

}  // namespace custom_dwa_local_planner

#endif  // CUSTOM_DWA_LOCAL_PLANNER_HPP_
