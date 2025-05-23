#include "pp_controller/pp_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
using namespace std;
namespace pp_controller
{

double calculate_angvel_forward(geometry_msgs::msg::PoseStamped goal_pose, geometry_msgs::msg::PoseStamped current_pose, const double max_linear_speed){
  const double wheel_base = 0.498;


  double tractor_yaw = tf2::impl::getYaw(tf2::Quaternion(
    current_pose.pose.orientation.x,
    current_pose.pose.orientation.y,
    current_pose.pose.orientation.z,
    current_pose.pose.orientation.w));

  double alpha = atan2(goal_pose.pose.position.y - current_pose.pose.position.y, 
                      goal_pose.pose.position.x - current_pose.pose.position.x);
  
  double heading_error = alpha - tractor_yaw;
  
  while (heading_error > M_PI) heading_error -= 2 * M_PI;
  while (heading_error < -M_PI) heading_error += 2 * M_PI;

  // Pure pursuit steering angle calculation
  double desired_angular_speed = max_linear_speed * tan(heading_error) / wheel_base;
  return desired_angular_speed;
}

double calculate_angvel_reverse(geometry_msgs::msg::PoseStamped goal_pose, geometry_msgs::msg::PoseStamped current_pose, const double max_linear_speed){
  const double wheel_base = 0.498;

  double tractor_yaw = tf2::impl::getYaw(tf2::Quaternion(
    current_pose.pose.orientation.x,
    current_pose.pose.orientation.y,
    current_pose.pose.orientation.z,
    current_pose.pose.orientation.w));

  double alpha = atan2(goal_pose.pose.position.y - current_pose.pose.position.y, 
                      goal_pose.pose.position.x - current_pose.pose.position.x);
  
  double heading_error = alpha - tractor_yaw;
  
  while (heading_error > M_PI) heading_error -= 2 * M_PI;
  while (heading_error < -M_PI) heading_error += 2 * M_PI;

  double desired_angular_speed = max_linear_speed * tan(heading_error) / wheel_base;
  return desired_angular_speed;
}

geometry_msgs::msg::PoseStamped calc_trailer_config(  geometry_msgs::msg::PoseStamped &tractor_pose, geometry_msgs::msg::PoseStamped &previous_trailer_pose, geometry_msgs::msg::PoseStamped &previous_tractor_pose, int dir){
  double distance = hypot(
    tractor_pose.pose.position.x - previous_tractor_pose.pose.position.x, 
    tractor_pose.pose.position.y - previous_tractor_pose.pose.position.y
  );

  const double speed = 0.4;
  double time = distance / speed;
  double new_trailer_yaw = 0.0;
  const double rtr = 0.5625;
  geometry_msgs::msg::PoseStamped trailer_pose;
  trailer_pose.header.stamp = tractor_pose.header.stamp;
  trailer_pose.header.frame_id = tractor_pose.header.frame_id;

  double parent_trailer_yaw = tf2::impl::getYaw(tf2::Quaternion(
    previous_trailer_pose.pose.orientation.x,
    previous_trailer_pose.pose.orientation.y,
    previous_trailer_pose.pose.orientation.z,
    previous_trailer_pose.pose.orientation.w));
  double parent_tractor_yaw = tf2::impl::getYaw(tf2::Quaternion(
    previous_tractor_pose.pose.orientation.x,
    previous_tractor_pose.pose.orientation.y,
    previous_tractor_pose.pose.orientation.z,
    previous_tractor_pose.pose.orientation.w));
  double current_tractor_yaw = tf2::impl::getYaw(tf2::Quaternion(
    tractor_pose.pose.orientation.x,
    tractor_pose.pose.orientation.y,
    tractor_pose.pose.orientation.z,
    tractor_pose.pose.orientation.w));

  const double trailer_offset = 0.4625;
  const double trailer_x_disp = 0.15;

  new_trailer_yaw = parent_trailer_yaw + (dir*speed/rtr)*sin(-parent_trailer_yaw + parent_tractor_yaw) * time;
  
  trailer_pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), new_trailer_yaw));

  trailer_pose.pose.position.x = tractor_pose.pose.position.x - trailer_offset * cos(current_tractor_yaw) - (trailer_x_disp * cos(new_trailer_yaw));
  trailer_pose.pose.position.y = tractor_pose.pose.position.y - trailer_offset * sin(current_tractor_yaw) - (trailer_x_disp * sin(new_trailer_yaw));

  return trailer_pose;
}

void PPController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
    )
{
    node_ = parent.lock();
    plugin_name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    is_initialized_ = false;
    last_waypoint_index_ = 0;
    trailer_plan_.header.stamp = node_->get_clock()->now();
    trailer_plan_.header.frame_id = "map";

    // Get parameters
    lookahead_distance_ = node_->declare_parameter(name + ".lookahead_distance", 1.0);
    max_linear_speed_ = node_->declare_parameter(name + ".max_linear_speed", 0.5);
    max_angular_speed_ = node_->declare_parameter(name + ".max_angular_speed", 1.0);

    // trailer position publisher
    trailer_position_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("/control_trailer_position", 10);

    RCLCPP_INFO(node_->get_logger(), "Pure Pursuit Controller configured");
}

void PPController::cleanup()
{
    trailer_position_publisher_.reset();
    RCLCPP_INFO(node_->get_logger(), "Cleaning up Pure Pursuit Controller");
}

void PPController::activate()
{
    RCLCPP_INFO(node_->get_logger(), "Activating Pure Pursuit Controller");
    trailer_position_publisher_->on_activate();
}

void PPController::deactivate()
{
    RCLCPP_INFO(node_->get_logger(), "Deactivating Pure Pursuit Controller");
    trailer_position_publisher_->on_deactivate();
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

  if (!is_initialized_) {
    first_waypoint_index = last_waypoint_index_;
  }

  // Find the point that is closest to the lookahead distance
  double min_distance_diff = std::numeric_limits<double>::max();
  int target_index = global_plan_.poses.size() - 1;  // Default to last point

  for (int i = first_waypoint_index; i < (int)global_plan_.poses.size(); ++i) {
    double distance = sqrt(pow(global_plan_.poses[i].pose.position.x - pose.pose.position.x, 2) + 
                          pow(global_plan_.poses[i].pose.position.y - pose.pose.position.y, 2));
    
    double distance_diff = abs(distance - lookahead_distance_);
    
    if (distance_diff < min_distance_diff) {
      min_distance_diff = distance_diff;
      target_index = i;
    }
    
    // If we've gone past the lookahead distance, we can stop searching
    if (distance > lookahead_distance_) {
      break;
    }
  }

  current_goal_pose = global_plan_.poses[target_index];
  last_waypoint_index_ = target_index;


  // Pure pursuit steering angle calculation
  double tractor_yaw = tf2::impl::getYaw(tf2::Quaternion(
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z,
    pose.pose.orientation.w));

  double alpha = atan2(current_goal_pose.pose.position.y - pose.pose.position.y, 
                      current_goal_pose.pose.position.x - pose.pose.position.x);
  
  double heading_error = alpha - tractor_yaw;
  
  while (heading_error > M_PI) heading_error -= 2 * M_PI;
  while (heading_error < -M_PI) heading_error += 2 * M_PI;

  double desired_angular_speed;
  double linear_speed;
  if (abs(heading_error) > M_PI/2) {
    linear_speed = -max_linear_speed_;
    desired_angular_speed = -calculate_angvel_forward(current_goal_pose, pose, max_linear_speed_);
  }else{
    linear_speed = max_linear_speed_;
    desired_angular_speed = calculate_angvel_forward(current_goal_pose, pose, max_linear_speed_);
  }

  cmd_vel.twist.linear.x = linear_speed;
  cmd_vel.twist.angular.z = std::clamp(desired_angular_speed, -max_angular_speed_, max_angular_speed_);

  is_initialized_ = false;
  return cmd_vel;
}

void PPController::setPlan(const nav_msgs::msg::Path & path)
{
    global_plan_ = path;
    is_initialized_ = true;
    last_waypoint_index_ = 0;
    // get trailer transform
    geometry_msgs::msg::TransformStamped transform_trailer;
    geometry_msgs::msg::TransformStamped transform_tractor;
    try {
        transform_trailer = tf_->lookupTransform(
        "map", "trailer_link", tf2::TimePointZero,
        tf2::durationFromSec(0.5));

        // Get transform from global frame to trailer_link
        transform_tractor = tf_->lookupTransform(
        "map", "base_link", tf2::TimePointZero,
        tf2::durationFromSec(0.5));
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(node_->get_logger(), "Transform error: %s", ex.what());
        return;
    }
    geometry_msgs::msg::PoseStamped current_trailer_pose;
    current_trailer_pose.pose.position.x = transform_trailer.transform.translation.x;
    current_trailer_pose.pose.position.y = transform_trailer.transform.translation.y;
    current_trailer_pose.pose.position.z = transform_trailer.transform.translation.z;
    current_trailer_pose.pose.orientation.x = transform_trailer.transform.rotation.x;
    current_trailer_pose.pose.orientation.y = transform_trailer.transform.rotation.y;
    current_trailer_pose.pose.orientation.z = transform_trailer.transform.rotation.z;
    current_trailer_pose.pose.orientation.w = transform_trailer.transform.rotation.w;
    trailer_plan_.poses.push_back(current_trailer_pose);
   
    geometry_msgs::msg::PoseStamped current_tractor_pose;
    current_tractor_pose.pose.position.x = transform_tractor.transform.translation.x;
    current_tractor_pose.pose.position.y = transform_tractor.transform.translation.y;
    current_tractor_pose.pose.position.z = transform_tractor.transform.translation.z;
    current_tractor_pose.pose.orientation.x = transform_tractor.transform.rotation.x;
    current_tractor_pose.pose.orientation.y = transform_tractor.transform.rotation.y;
    current_tractor_pose.pose.orientation.z = transform_tractor.transform.rotation.z;
    current_tractor_pose.pose.orientation.w = transform_tractor.transform.rotation.w;

    for (int i = 1; i < (int)global_plan_.poses.size(); i++) {
      // calculaton of the direction, either reverse or foward
      double current_tractor_yaw = tf2::impl::getYaw(tf2::Quaternion(
        current_tractor_pose.pose.orientation.x,
        current_tractor_pose.pose.orientation.y,
        current_tractor_pose.pose.orientation.z,
        current_tractor_pose.pose.orientation.w));

      double alpha = atan2( global_plan_.poses[i].pose.position.y - current_tractor_pose.pose.position.y, 
                          global_plan_.poses[i].pose.position.x - current_tractor_pose.pose.position.x);
      
      double heading_error = alpha - current_tractor_yaw;

      while (heading_error > M_PI) heading_error -= 2 * M_PI;
      while (heading_error < -M_PI) heading_error += 2 * M_PI;

      int dir = 1;
      if (abs(heading_error) > M_PI/2) {
        dir = -1;
      }

      geometry_msgs::msg::PoseStamped trailer_pose;
      trailer_pose = calc_trailer_config(global_plan_.poses[i], current_trailer_pose, current_tractor_pose, dir);
      trailer_plan_.poses.push_back(trailer_pose);
      current_trailer_pose = trailer_pose;
      current_tractor_pose = global_plan_.poses[i];
    }
    if (trailer_position_publisher_->is_activated()) {
      cout << "publishing trailer plan" << endl;
      
      for (int i = 0; i < (int)trailer_plan_.poses.size(); i++) {
        cout << "global frame: " << trailer_plan_.poses[i].header.frame_id << endl;
        cout << "trailer pose: " << trailer_plan_.poses[i].pose.position.x << ", " << trailer_plan_.poses[i].pose.position.y << endl;
      }
      trailer_position_publisher_->publish(trailer_plan_);
    }else{
      RCLCPP_INFO(node_->get_logger(), "Trailer position publisher is not activated");
    }
}

void PPController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
    (void)speed_limit;
    (void)percentage;
}

} // namespace pp_controller

PLUGINLIB_EXPORT_CLASS(pp_controller::PPController, nav2_core::Controller)

