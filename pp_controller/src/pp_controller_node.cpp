#include "pp_controller/pp_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
using namespace std;
namespace pp_controller
{

geometry_msgs::msg::PoseStamped PPController::from_odom_to_map(geometry_msgs::msg::PoseStamped &pose){
  geometry_msgs::msg::TransformStamped transform;
  geometry_msgs::msg::PoseStamped pose_map;
  try{
    transform = tf_->lookupTransform("map", "odom", tf2::TimePointZero, tf2::durationFromSec(0.5));
  }catch(const tf2::TransformException & ex){
    RCLCPP_ERROR(node_->get_logger(), "Transform error: %s", ex.what());
    return pose_map;
  }
  tf2::doTransform(pose, pose_map, transform);
  pose_map.header.frame_id = "map";
  return pose_map;
}

geometry_msgs::msg::PoseStamped PPController::from_map_to_odom(geometry_msgs::msg::PoseStamped &pose){
  geometry_msgs::msg::TransformStamped transform;
  geometry_msgs::msg::PoseStamped pose_odom;
  try{
    transform = tf_->lookupTransform("odom", "map", tf2::TimePointZero, tf2::durationFromSec(0.5));
  }catch(const tf2::TransformException & ex){
    RCLCPP_ERROR(node_->get_logger(), "Transform error: %s", ex.what());
    return pose_odom;
  }
  tf2::doTransform(pose, pose_odom, transform);
  pose_odom.header.frame_id = "odom";
  return pose_odom;
}

geometry_msgs::msg::PointStamped PPController::point_from_odom_to_map(geometry_msgs::msg::PoseStamped &pose){
  geometry_msgs::msg::PointStamped point_map;
  geometry_msgs::msg::TransformStamped transform;
  try{
    transform = tf_->lookupTransform("map", "odom", tf2::TimePointZero, tf2::durationFromSec(0.5));
  }catch(const tf2::TransformException & ex){
    RCLCPP_ERROR(node_->get_logger(), "Transform error: %s", ex.what());
    return point_map;
  }
  
  // Create a PointStamped from the pose position
  geometry_msgs::msg::PointStamped point_odom;
  point_odom.header = pose.header;
  point_odom.point = pose.pose.position;
  
  tf2::doTransform(point_odom, point_map, transform);
  point_map.header.frame_id = "map";
  return point_map;
}

geometry_msgs::msg::PointStamped PPController::point_from_map_to_odom(geometry_msgs::msg::PointStamped &point){
  geometry_msgs::msg::PointStamped point_odom;
  geometry_msgs::msg::TransformStamped transform;
  try{
    transform = tf_->lookupTransform("odom", "map", tf2::TimePointZero, tf2::durationFromSec(0.5));
  }catch(const tf2::TransformException & ex){
    RCLCPP_ERROR(node_->get_logger(), "Transform error: %s", ex.what());
    return point_odom;
  }
  tf2::doTransform(point, point_odom, transform);
  point_odom.header.frame_id = "odom";
  return point_odom;
}
double PPController::calculate_angvel_reverse(geometry_msgs::msg::PoseStamped goal_trailer_pose, geometry_msgs::msg::PoseStamped current_trailer_pose, geometry_msgs::msg::PoseStamped current_tractor_pose,const double max_linear_speed, double lookahead_distance){
  const double rtr = 0.5625;
  const double k = 2.0;

  double parent_trailer_yaw = tf2::getYaw(current_trailer_pose.pose.orientation);
  
  double alpha = atan2(goal_trailer_pose.pose.position.y - current_trailer_pose.pose.position.y, 
                      goal_trailer_pose.pose.position.x - current_trailer_pose.pose.position.x);
  double heading_error = alpha - parent_trailer_yaw;
  while (heading_error > M_PI) heading_error -= 2 * M_PI;
  while (heading_error < -M_PI) heading_error += 2 * M_PI;
  double t0 = tf2::getYaw(current_tractor_pose.pose.orientation);
  double t1 = t0 - parent_trailer_yaw;
  while (t1 > M_PI) t1 -= 2 * M_PI;
  while (t1 < -M_PI) t1 += 2 * M_PI;

  double hitch_angle = atan2(2*rtr*sin(heading_error), lookahead_distance);

  double desired_angular_speed = max_linear_speed * (-k * (t1 - hitch_angle) - (sin(t1)/rtr));
  // desired_angular_speed = 0.0;
  // RCLCPP_INFO(node_->get_logger(), "desired_angular_speed reverse: %f", desired_angular_speed);

  return desired_angular_speed;
}
double PPController::calculate_angvel_forward(geometry_msgs::msg::PoseStamped goal_pose, geometry_msgs::msg::PoseStamped current_pose, double lookahead_distance){
  const double wheel_base = 0.498;
  const double turn_gain = 0.75;


  double tractor_yaw = tf2::getYaw(current_pose.pose.orientation);

  double alpha = atan2(goal_pose.pose.position.y - current_pose.pose.position.y, 
                      goal_pose.pose.position.x - current_pose.pose.position.x);
  
  double heading_error = alpha - tractor_yaw;
  
  while (heading_error > M_PI) heading_error -= 2 * M_PI;
  while (heading_error < -M_PI) heading_error += 2 * M_PI;

  // Pure pursuit steering angle calculation
  double desired_angular_speed = turn_gain*atan2(2*wheel_base*sin(heading_error), lookahead_distance);
  // RCLCPP_INFO(node_->get_logger(), "desired_angular_speed forward: %f", desired_angular_speed);
  return desired_angular_speed;
}

geometry_msgs::msg::PoseStamped PPController::calc_trailer_config(  geometry_msgs::msg::PoseStamped &tractor_pose, geometry_msgs::msg::PoseStamped &previous_trailer_pose, geometry_msgs::msg::PoseStamped &previous_tractor_pose, int dir){
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

  double parent_trailer_yaw = tf2::getYaw(previous_trailer_pose.pose.orientation);
  double parent_tractor_yaw = tf2::getYaw(previous_tractor_pose.pose.orientation);

  double current_tractor_yaw = tf2::getYaw(tractor_pose.pose.orientation);
  // cout << "cuurent trailer yaw: " << parent_trailer_yaw << endl;
  const double trailer_offset = 0.4625;
  const double trailer_x_disp = 0.15 + 0.55/4 + 0.55/2;

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

    global_plan_.header.frame_id = "map";

    // Get parameters
    lookahead_distance_ = node_->declare_parameter(name + ".lookahead_distance", 1.0);
    max_linear_speed_ = node_->declare_parameter(name + ".max_linear_speed", 0.5);
    max_angular_speed_ = node_->declare_parameter(name + ".max_angular_speed", 1.0);
    collision_check_distance_ = node_->declare_parameter(name + ".collision_check_distance", 0.5);
    use_collision_check_ = node_->declare_parameter(name + ".use_collision_check", false);

    // trailer position publisher
    trailer_position_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("/control_trailer_position", 10);
    control_goal_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/control_goal_pose", 10);
    goal_trailer_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_trailer_pose", 10);
    collision_check_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("/collision_check_areas", 10);
    global_plan_map_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("/global_plan_map", 10);

    RCLCPP_INFO(node_->get_logger(), "Pure Pursuit Controller configured");
}

void PPController::cleanup()
{
    trailer_position_publisher_.reset();
    control_goal_pose_publisher_.reset();
    goal_trailer_publisher_.reset();
    collision_check_publisher_.reset();
    global_plan_map_publisher_.reset();
    RCLCPP_INFO(node_->get_logger(), "Cleaning up Pure Pursuit Controller");
}

void PPController::activate()
{
    RCLCPP_INFO(node_->get_logger(), "Activating Pure Pursuit Controller");
    trailer_position_publisher_->on_activate();
    control_goal_pose_publisher_->on_activate();
    goal_trailer_publisher_->on_activate();
    collision_check_publisher_->on_activate();
    global_plan_map_publisher_->on_activate();
    // print the parameters
    RCLCPP_INFO(node_->get_logger(), "Lookahead distance: %f", lookahead_distance_);
    RCLCPP_INFO(node_->get_logger(), "Max linear speed: %f", max_linear_speed_);
    RCLCPP_INFO(node_->get_logger(), "Max angular speed: %f", max_angular_speed_);
    RCLCPP_INFO(node_->get_logger(), "Collision check distance: %f", collision_check_distance_);
    RCLCPP_INFO(node_->get_logger(), "Use collision check: %d", use_collision_check_);
}

void PPController::deactivate()
{
    RCLCPP_INFO(node_->get_logger(), "Deactivating Pure Pursuit Controller");
    trailer_position_publisher_->on_deactivate();
    control_goal_pose_publisher_->on_deactivate();
    goal_trailer_publisher_->on_deactivate();
    collision_check_publisher_->on_deactivate();
    global_plan_map_publisher_->on_deactivate();
}

bool PPController::checkForCollisions(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose)
{
    // Get the costmap
    auto costmap = costmap_ros_->getCostmap();
    
    // Robot footprint dimensions in meters
    const double robot_width = 0.7;  // meters
    const double robot_length = 1.0; // meters

    // Calculate the direction from current pose to goal pose (fixed direction for entire check)
    double goal_direction = std::atan2(
        goal_pose.pose.position.y - current_pose.pose.position.y,
        goal_pose.pose.position.x - current_pose.pose.position.x
    );

    // Check 3 meters in front of the robot in the direction of the goal
    const double check_distance = 1.0;  // 3 meters
    const double step_size = 0.1;  // Check every 10cm
    int num_points = static_cast<int>(check_distance / step_size);
    
    // Create a path for the collision check positions
    nav_msgs::msg::Path collision_check_path;
    collision_check_path.header.stamp = node_->get_clock()->now();
    collision_check_path.header.frame_id = "map";
    
    // Calculate the direction vectors
    double goal_dir_x = std::cos(goal_direction);
    double goal_dir_y = std::sin(goal_direction);
    double perp_x = -std::sin(goal_direction);
    double perp_y = std::cos(goal_direction);
    
    // Robot footprint dimensions
    // double half_width = robot_width / 2.0;
    double half_length = robot_length / 2.0;
    
    bool collision_detected = false;
    
    // Check for collisions along the path and create path points
    for (int i = 0; i <= num_points; ++i) {
        double distance_along_path = i * step_size;
        
        // Move the robot center along the goal direction
        double center_x = current_pose.pose.position.x + distance_along_path * goal_dir_x;
        double center_y = current_pose.pose.position.y + distance_along_path * goal_dir_y;
        
        // Calculate the front center position of the rectangle (middle front)
        double front_center_x = center_x + half_length * goal_dir_x;
        double front_center_y = center_y + half_length * goal_dir_y;
        
        // Create pose for the front center position
        geometry_msgs::msg::PoseStamped front_center_pose;
        front_center_pose.header.stamp = node_->get_clock()->now();
        front_center_pose.header.frame_id = "map";
        front_center_pose.pose.position.x = front_center_x;
        front_center_pose.pose.position.y = front_center_y;
        front_center_pose.pose.position.z = 0.0;
        
        // Set orientation to point in the goal direction
        front_center_pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), goal_direction));
        
        
        collision_check_path.poses.push_back(front_center_pose);
        
        // Check for collisions
        for (double w = -robot_width/2; w <= robot_width/2; w += 0.1) {
            for (double l = -robot_length/2; l <= robot_length/2; l += 0.1) {
                double check_x = center_x + w * goal_dir_x - l * perp_x;
                double check_y = center_y + w * goal_dir_y + l * perp_y;
                
                unsigned int map_x, map_y;
                //pass the check_x and y from map to odom and then preform the check
                geometry_msgs::msg::PointStamped point_map;
                point_map.header.stamp = node_->get_clock()->now();
                point_map.header.frame_id = "map";
                point_map.point.x = check_x;
                point_map.point.y = check_y;
                point_map.point.z = 0.0;
                geometry_msgs::msg::PointStamped point_odom = point_from_map_to_odom(point_map);
                if (!costmap->worldToMap(point_odom.point.x, point_odom.point.y, map_x, map_y)) {
                    RCLCPP_WARN(node_->get_logger(), "Point not in costmap: (%.2f, %.2f)", point_odom.point.x, point_odom.point.y);
                    continue;  
                }
                
                unsigned char cost = costmap->getCost(map_x, map_y);
                
                if (cost > 253) {
                    RCLCPP_WARN(node_->get_logger(), 
                        "Collision detected at world pos (%.2f, %.2f) map pos (%d, %d) with cost %d (threshold: %d) at distance %.2f m", 
                        check_x, check_y, map_x, map_y, cost, 253, distance_along_path);
                    collision_detected = true;
                }
            }
        }
    }
    
    // Publish the collision check path
    if (collision_check_publisher_->is_activated()) {
        collision_check_publisher_->publish(collision_check_path);
    }
    
    return collision_detected;
}

geometry_msgs::msg::TwistStamped PPController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)velocity;
  (void)goal_checker;
  geometry_msgs::msg::TwistStamped cmd_vel;
  geometry_msgs::msg::TransformStamped transform_trailer;
  geometry_msgs::msg::TransformStamped transform_tractor;
  geometry_msgs::msg::PoseStamped pose_map;
  try {
      transform_trailer = tf_->lookupTransform(
      "map", "trailer_link", tf2::TimePointZero,
      tf2::durationFromSec(0.5));
      transform_tractor = tf_->lookupTransform(
        "map", "base_link", tf2::TimePointZero, 
        tf2::durationFromSec(0.5));
      pose_map.pose.position.x = transform_tractor.transform.translation.x;
      pose_map.pose.position.y = transform_tractor.transform.translation.y;
      pose_map.pose.position.z = transform_tractor.transform.translation.z;
      pose_map.pose.orientation.x = transform_tractor.transform.rotation.x;
      pose_map.pose.orientation.y = transform_tractor.transform.rotation.y;
      pose_map.pose.orientation.z = transform_tractor.transform.rotation.z;
      pose_map.pose.orientation.w = transform_tractor.transform.rotation.w;
      pose_map.header.stamp = node_->get_clock()->now();
      pose_map.header.frame_id = "map";
  } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(node_->get_logger(), "Transform error: %s", ex.what());
      return cmd_vel;
  }

  cmd_vel.header.stamp = node_->get_clock()->now();
  cmd_vel.header.frame_id = "base_link";

  geometry_msgs::msg::PoseStamped current_trailer_pose;
  current_trailer_pose.header.stamp = node_->get_clock()->now();
  current_trailer_pose.header.frame_id = "map";
  current_trailer_pose.pose.position.x = transform_trailer.transform.translation.x;
  current_trailer_pose.pose.position.y = transform_trailer.transform.translation.y;
  current_trailer_pose.pose.position.z = transform_trailer.transform.translation.z;
  current_trailer_pose.pose.orientation.x = transform_trailer.transform.rotation.x;
  current_trailer_pose.pose.orientation.y = transform_trailer.transform.rotation.y;
  current_trailer_pose.pose.orientation.z = transform_trailer.transform.rotation.z;
  current_trailer_pose.pose.orientation.w = transform_trailer.transform.rotation.w;
  int first_waypoint_index = 0;

  if (!is_initialized_) {
    first_waypoint_index = last_waypoint_index_;
  }

  // Find the point that is closest to the lookahead distance
  double min_distance_diff = std::numeric_limits<double>::max();
  int target_index = global_plan_.poses.size() - 1;  // Default to last point

  for (int i = first_waypoint_index; i < (int)global_plan_.poses.size(); ++i) {
    double distance = sqrt(pow(global_plan_.poses[i].pose.position.x - pose_map.pose.position.x, 2) + 
                          pow(global_plan_.poses[i].pose.position.y - pose_map.pose.position.y, 2));
    
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

  
  geometry_msgs::msg::PoseStamped current_goal_trailer_pose;
  geometry_msgs::msg::PoseStamped current_goal_pose;
  current_goal_pose = global_plan_.poses[target_index];
  current_goal_trailer_pose = trailer_plan_.poses[target_index];
  last_waypoint_index_ = target_index;
  current_goal_trailer_pose.header.stamp = node_->get_clock()->now();
  current_goal_trailer_pose.header.frame_id = "map";
  
  if (control_goal_pose_publisher_->is_activated()) {
    control_goal_pose_publisher_->publish(current_goal_pose);
  }
  if (goal_trailer_publisher_->is_activated()) {
    goal_trailer_publisher_->publish(current_goal_trailer_pose);
  }

  // Check for collisions before proceeding
  if (checkForCollisions(pose_map, current_goal_pose)) {
    RCLCPP_WARN(node_->get_logger(), "Collision detected! Stopping robot.");
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }

  // Pure pursuit steering angle calculation
  double tractor_yaw = tf2::getYaw(pose_map.pose.orientation);

  double alpha = atan2(current_goal_pose.pose.position.y - pose_map.pose.position.y, 
                      current_goal_pose.pose.position.x - pose_map.pose.position.x);
  
  double heading_error = alpha - tractor_yaw;
  
  while (heading_error > M_PI) heading_error -= 2 * M_PI;
  while (heading_error < -M_PI) heading_error += 2 * M_PI;

  double desired_angular_speed;
  double linear_speed;
  if (abs(heading_error) > M_PI/2) {
    linear_speed = -max_linear_speed_;
    desired_angular_speed = calculate_angvel_reverse(current_goal_trailer_pose, current_trailer_pose, pose_map, max_linear_speed_, lookahead_distance_);
  }else{
    linear_speed = max_linear_speed_;
    desired_angular_speed = calculate_angvel_forward(current_goal_pose, pose_map, lookahead_distance_);
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
    trailer_plan_.poses.clear();
    trailer_plan_.header.stamp = node_->get_clock()->now();
    trailer_plan_.header.frame_id = "map";

    // get trailer transform
    geometry_msgs::msg::TransformStamped transform_trailer;
    try {
        transform_trailer = tf_->lookupTransform(
        "map", "trailer_link", tf2::TimePointZero,
        tf2::durationFromSec(0.5));


    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(node_->get_logger(), "Transform error: %s", ex.what());
        return;
    }
    geometry_msgs::msg::PoseStamped current_trailer_pose;
    const double offset_from_link = 0.55/2 + 0.55/4;
    double current_trailer_yaw = tf2::getYaw(transform_trailer.transform.rotation);
    current_trailer_pose.pose.position.x = transform_trailer.transform.translation.x - offset_from_link * cos(current_trailer_yaw);
    current_trailer_pose.pose.position.y = transform_trailer.transform.translation.y - offset_from_link * sin(current_trailer_yaw);
    current_trailer_pose.pose.position.z = 0;
    current_trailer_pose.pose.orientation.x = transform_trailer.transform.rotation.x ;
    current_trailer_pose.pose.orientation.y = transform_trailer.transform.rotation.y;
    current_trailer_pose.pose.orientation.z = transform_trailer.transform.rotation.z;
    current_trailer_pose.pose.orientation.w = transform_trailer.transform.rotation.w;
    // print trailer yaw
    trailer_plan_.poses.push_back(current_trailer_pose);
   

    for (int i = 1; i < (int)global_plan_.poses.size(); i++) {
      // calculaton of the direction, either reverse or foward
      geometry_msgs::msg::PoseStamped current_tractor_pose = global_plan_.poses[i-1];
      double current_tractor_yaw = tf2::getYaw(tf2::Quaternion(
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
    }
    if (trailer_position_publisher_->is_activated()) {
      trailer_position_publisher_->publish(trailer_plan_);
    }else{
      RCLCPP_INFO(node_->get_logger(), "Trailer position publisher is not activated");
    }
    
    // Publish the global plan in map frame
    //publishGlobalPlanMap();
}

void PPController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
    (void)speed_limit;
    (void)percentage;
}

void PPController::publishGlobalPlanMap()
{
   
    
    global_plan_map_publisher_->publish(global_plan_);
}

} // namespace pp_controller

PLUGINLIB_EXPORT_CLASS(pp_controller::PPController, nav2_core::Controller)


