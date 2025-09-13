#ifndef CUSTOM_DWA_LOCAL_PLANNER_HPP_
#define CUSTOM_DWA_LOCAL_PLANNER_HPP_

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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

protected:
    // DWA Parameters
    double sim_time_;
    double max_speed_x_, min_speed_x_;
    double max_speed_theta_, min_speed_theta_;
    double acc_lim_x_, acc_lim_theta_;
    int v_samples_, w_samples_;

    // Cost function weights
    double goal_dist_scale_;
    double path_dist_scale_;
    double occdist_scale_;
    double heading_scale_;
    double progress_scale_;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
  rclcpp::Logger logger_{rclcpp::get_logger("CustomDWAPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  // Visualization Publisher
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_pub_;
};

}  // namespace custom_dwa_local_planner

#endif  // CUSTOM_DWA_LOCAL_PLANNER_HPP_

