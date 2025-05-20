#include "pp_controller/pp_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
using namespace std;
namespace pp_controller
{

void PPController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent.lock();
    plugin_name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    is_initialized_ = false;
    last_waypoint_index_ = 0;

    // Get parameters
    lookahead_distance_ = node_->declare_parameter(name + ".lookahead_distance", 1.0);
    max_linear_speed_ = node_->declare_parameter(name + ".max_linear_speed", 0.5);
    max_angular_speed_ = node_->declare_parameter(name + ".max_angular_speed", 1.0);

    RCLCPP_INFO(node_->get_logger(), "Pure Pursuit Controller configured");
}

void PPController::cleanup()
{
    RCLCPP_INFO(node_->get_logger(), "Cleaning up Pure Pursuit Controller");
}

void PPController::activate()
{
    RCLCPP_INFO(node_->get_logger(), "Activating Pure Pursuit Controller");
}

void PPController::deactivate()
{
    RCLCPP_INFO(node_->get_logger(), "Deactivating Pure Pursuit Controller");
}

geometry_msgs::msg::TwistStamped PPController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)velocity;
  (void)goal_checker;
  const double wheel_base = 0.498;

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = node_->get_clock()->now();
  cmd_vel.header.frame_id = "base_link";

  geometry_msgs::msg::PoseStamped current_goal_pose;
  int first_waypoint_index = 0;

  // if (!is_initialized_){
  //   first_waypoint_index = last_waypoint_index_;
  // }

  // double last_distance = sqrt(pow(global_plan_.poses[first_waypoint_index].pose.position.x - pose.pose.position.x, 2) + 
  //                           pow(global_plan_.poses[first_waypoint_index].pose.position.y - pose.pose.position.y, 2));
  // for (int i = first_waypoint_index+1; i < (int)global_plan_.poses.size(); ++i){
  //   current_goal_pose = global_plan_.poses[i];
  //   double distance = sqrt(pow(current_goal_pose.pose.position.x - pose.pose.position.x, 2) + 
  //                         pow(current_goal_pose.pose.position.y - pose.pose.position.y, 2));
  //   if (distance > last_distance && distance < lookahead_distance_){
  //     last_waypoint_index_ = i;
  //     break;
  //   }
  // }
  current_goal_pose = global_plan_.poses[global_plan_.poses.size() - 1];

  double yaw = tf2::impl::getYaw(tf2::Quaternion(
    current_goal_pose.pose.orientation.x,
    current_goal_pose.pose.orientation.y,
    current_goal_pose.pose.orientation.z,
    current_goal_pose.pose.orientation.w));
  double tractor_yaw = tf2::impl::getYaw(tf2::Quaternion(
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z,
    pose.pose.orientation.w));
  double error = yaw - tractor_yaw;
  double theta = atan2(current_goal_pose.pose.position.y - pose.pose.position.y, current_goal_pose.pose.position.x - pose.pose.position.x) - error;
  double desired_angular_speed = atan2(2*wheel_base*sin(theta), lookahead_distance_);
  cout << "current_goal_pose: " << current_goal_pose.pose.position.x << ", " << current_goal_pose.pose.position.y << endl;
  cout << "theta: " << theta << endl;
  cout << "desired_angular_speed: " << desired_angular_speed << endl;
  cmd_vel.twist.linear.x = max_linear_speed_;
  cmd_vel.twist.angular.z = std::clamp(desired_angular_speed, -max_angular_speed_, max_angular_speed_);

  is_initialized_ = false;
  return cmd_vel;
}

void PPController::setPlan(const nav_msgs::msg::Path & path)
{
    global_plan_ = path;
    is_initialized_ = true;
    last_waypoint_index_ = 0;
}

void PPController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
    (void)speed_limit;
    (void)percentage;
}

} // namespace pp_controller

PLUGINLIB_EXPORT_CLASS(pp_controller::PPController, nav2_core::Controller)

