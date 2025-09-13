#include "custom_dwa_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "angles/angles.h"
#include <limits>

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
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  // Get DWA parameters from the parameter server
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".sim_time", rclcpp::ParameterValue(1.0));
  node->get_parameter(plugin_name_ + ".sim_time", sim_time_);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_speed_x", rclcpp::ParameterValue(0.22));
  node->get_parameter(plugin_name_ + ".max_speed_x", max_speed_x_);
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".min_speed_x", rclcpp::ParameterValue(0.0));
  node->get_parameter(plugin_name_ + ".min_speed_x", min_speed_x_);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_speed_theta", rclcpp::ParameterValue(1.0));
  node->get_parameter(plugin_name_ + ".max_speed_theta", max_speed_theta_);
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".min_speed_theta", rclcpp::ParameterValue(-1.0));
  node->get_parameter(plugin_name_ + ".min_speed_theta", min_speed_theta_);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".acc_lim_x", rclcpp::ParameterValue(0.5));
  node->get_parameter(plugin_name_ + ".acc_lim_x", acc_lim_x_);
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".acc_lim_theta", rclcpp::ParameterValue(1.5));
  node->get_parameter(plugin_name_ + ".acc_lim_theta", acc_lim_theta_);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".v_samples", rclcpp::ParameterValue(20));
  node->get_parameter(plugin_name_ + ".v_samples", v_samples_);
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".w_samples", rclcpp::ParameterValue(40));
  node->get_parameter(plugin_name_ + ".w_samples", w_samples_);
  
  // Weights for the cost function
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_dist_scale", rclcpp::ParameterValue(24.0));
  node->get_parameter(plugin_name_ + ".goal_dist_scale", goal_dist_scale_);
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".path_dist_scale", rclcpp::ParameterValue(32.0));
  node->get_parameter(plugin_name_ + ".path_dist_scale", path_dist_scale_);
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".occdist_scale", rclcpp::ParameterValue(0.01));
  node->get_parameter(plugin_name_ + ".occdist_scale", occdist_scale_);
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".heading_scale", rclcpp::ParameterValue(15.0));
  node->get_parameter(plugin_name_ + ".heading_scale", heading_scale_);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".progress_scale", rclcpp::ParameterValue(20.0));
  node->get_parameter(plugin_name_ + ".progress_scale", progress_scale_); 

  // Create publisher for visualizing trajectories
  traj_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("local_trajectories", 1);
}

void CustomDWAPlanner::activate() {
  RCLCPP_INFO(logger_, "Activating plugin %s", plugin_name_.c_str());
  traj_pub_->on_activate();
}

void CustomDWAPlanner::deactivate() {
  RCLCPP_INFO(logger_, "Deactivating plugin %s", plugin_name_.c_str());
  traj_pub_->on_deactivate();
}

void CustomDWAPlanner::cleanup() {
  RCLCPP_INFO(logger_, "Cleaning up plugin %s", plugin_name_.c_str());
  traj_pub_.reset();
}

void CustomDWAPlanner::setPlan(const nav_msgs::msg::Path & path) {
  global_plan_ = path;
}

geometry_msgs::msg::TwistStamped CustomDWAPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  if (global_plan_.poses.empty()) {
    RCLCPP_WARN(logger_, "Global plan is empty, stopping robot");
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }

  // 1. DYNAMIC WINDOW CALCULATION
  double min_v = std::max(min_speed_x_, velocity.linear.x - acc_lim_x_ * sim_time_);
  double max_v = std::min(max_speed_x_, velocity.linear.x + acc_lim_x_ * sim_time_);
  double min_w = std::max(min_speed_theta_, velocity.angular.z - acc_lim_theta_ * sim_time_);
  double max_w = std::min(max_speed_theta_, velocity.angular.z + acc_lim_theta_ * sim_time_);

  double v_step = (max_v - min_v) / v_samples_;
  double w_step = (max_w - min_w) / w_samples_;
  
  double best_cost = std::numeric_limits<double>::max();
  double best_v = 0.0, best_w = 0.0;

  visualization_msgs::msg::MarkerArray markers;
  int marker_id = 0;

  auto closest_pose_it = std::min_element(global_plan_.poses.begin(), global_plan_.poses.end(),
    [&](const auto& p1, const auto& p2) {
      return std::hypot(p1.pose.position.x - pose.pose.position.x, p1.pose.position.y - pose.pose.position.y) <
             std::hypot(p2.pose.position.x - pose.pose.position.x, p2.pose.position.y - pose.pose.position.y);
  });

  geometry_msgs::msg::PoseStamped lookahead_pose = *closest_pose_it;
  double lookahead_dist = 0.5; // meters
  auto lookahead_it = closest_pose_it;
  while (lookahead_it != global_plan_.poses.end() &&
         std::hypot(lookahead_it->pose.position.x - closest_pose_it->pose.position.x,
                    lookahead_it->pose.position.y - closest_pose_it->pose.position.y) < lookahead_dist) {
    lookahead_pose = *lookahead_it;
    ++lookahead_it;
  }

  double current_dist_to_lookahead = std::hypot(pose.pose.position.x - lookahead_pose.pose.position.x,
                                                pose.pose.position.y - lookahead_pose.pose.position.y);

  // 2. TRAJECTORY SAMPLING AND EVALUATION
  for (double v = min_v; v <= max_v; v += v_step) {
    if (v_step == 0.0) { v = max_v; }
    for (double w = min_w; w <= max_w; w += w_step) {
      if (w_step == 0.0) { w = max_w; }

      std::vector<geometry_msgs::msg::Pose2D> trajectory;
      geometry_msgs::msg::Pose2D current_pose;
      current_pose.x = 0; current_pose.y = 0; current_pose.theta = 0;
      double dt = sim_time_ / 10.0;
      for (double t = 0; t < sim_time_; t += dt) {
        current_pose.x += v * cos(current_pose.theta) * dt;
        current_pose.y += v * sin(current_pose.theta) * dt;
        current_pose.theta += w * dt;
        trajectory.push_back(current_pose);
      }
      
      // 4. COST FUNCTION EVALUATION
      double occ_cost = 0.0;
      for(const auto& p : trajectory) {
          double world_x = pose.pose.position.x + p.x * cos(tf2::getYaw(pose.pose.orientation)) - p.y * sin(tf2::getYaw(pose.pose.orientation));
          double world_y = pose.pose.position.y + p.x * sin(tf2::getYaw(pose.pose.orientation)) + p.y * cos(tf2::getYaw(pose.pose.orientation));
          unsigned int mx, my;
          if (costmap_ros_->getCostmap()->worldToMap(world_x, world_y, mx, my)) {
              unsigned char cost = costmap_ros_->getCostmap()->getCost(mx, my);
              if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
                  occ_cost = std::numeric_limits<double>::max();
                  break;
              }
          }
      }
      if (occ_cost > 0) { continue; }

      double final_x = pose.pose.position.x + trajectory.back().x * cos(tf2::getYaw(pose.pose.orientation)) - trajectory.back().y * sin(tf2::getYaw(pose.pose.orientation));
      double final_y = pose.pose.position.y + trajectory.back().x * sin(tf2::getYaw(pose.pose.orientation)) + trajectory.back().y * cos(tf2::getYaw(pose.pose.orientation));

      double goal_dist_cost = std::hypot(final_x - lookahead_pose.pose.position.x, final_y - lookahead_pose.pose.position.y);
      double path_dist_cost = std::hypot(final_x - closest_pose_it->pose.position.x, final_y - closest_pose_it->pose.position.y);

      double angle_to_lookahead = atan2(lookahead_pose.pose.position.y - pose.pose.position.y, lookahead_pose.pose.position.x - pose.pose.position.x);
      double final_global_theta = tf2::getYaw(pose.pose.orientation) + trajectory.back().theta;
      double heading_cost = std::abs(angles::shortest_angular_distance(final_global_theta, angle_to_lookahead));
      
      // A negative value means the trajectory moved closer, positive means it moved farther.
      double progress_cost = goal_dist_cost - current_dist_to_lookahead;

      double cost = progress_scale_ * progress_cost +
                    goal_dist_scale_ * goal_dist_cost +
                    path_dist_scale_ * path_dist_cost +
                    heading_scale_ * heading_cost;
      
      if (cost < best_cost) {
        best_cost = cost;
        best_v = v;
        best_w = w;
      }
      // Visualization
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = costmap_ros_->getBaseFrameID();
      marker.header.stamp = clock_->now();
      marker.ns = "trajectories";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.01;
      marker.color.a = 1.0;
      marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; // Green for valid
      for (const auto& p : trajectory) {
          geometry_msgs::msg::Point pt;
          pt.x = p.x; pt.y = p.y; pt.z = 0;
          marker.points.push_back(pt);
      }
      markers.markers.push_back(marker);
      if (v_step == 0.0) break;
    }
    if (w_step == 0.0) break;
  }
  traj_pub_->publish(markers);

  // 5. RETURN BEST VELOCITY
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd_vel.twist.linear.x = best_v;
  cmd_vel.twist.angular.z = best_w;
  
  return cmd_vel;
}


void CustomDWAPlanner::setSpeedLimit(const double & /*speed_limit*/, const bool & /*percentage*/)
{
  // dynamically change max_speed_x_ 
}

}  // namespace custom_dwa_local_planner

PLUGINLIB_EXPORT_CLASS(custom_dwa_local_planner::CustomDWAPlanner, nav2_core::Controller)