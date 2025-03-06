#include <cmath>
#include <string>
#include <memory>
#include <queue>
#include <unordered_map>
#include "nav2_util/node_utils.hpp"
#include "astar_planner/astar_planner.hpp"

double calculateDist(int x1, int y1, int x2, int y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}


namespace astar_planner
{

  struct node_ 
  {
    int x, y;
    Node *parent
    double f, g, h;
  };

void Astar::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void Astar::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void Astar::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void Astar::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path Astar::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  // calculating the number of loops for current value of interpolation_resolution_
  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  //array of size numberofcells of custom type Node
  const int numberofcells = costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY();
  Node nodes[numberofcells];
  bool is_goal_reached = false;
  Node current_node ={
    .x = mx,
    .y = my,
    .f = calculateDist(start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y),
    .g = 0,
    .h = calculateDist(start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y),
    .parent = NULL
  };
  int numnodes = 0;
  nodes[numnodes] = current_node;
  numnodes++;

  while (!is_goal_reached){
    //find the node with the smallest f value
    for (int i = -1; i < 2; i++){
      int curx = current_node.x + i;
      int cury = current_node.y + i;

      worldtoMap(curx, cury, mx, my);
      double h = calculateDist(curx, cury, goal.pose.position.x, goal.pose.position.y);
      double g = current_node.g + calculateDist(curx, cury, current_node.x, current_node.y);
      double f = g + h;
      if (current_node.parent != NULL){
        if (f < current_node.parent->f){
          current_node.parent = &nodes[numnodes];
        }
      }
      Node current_node ={
        .x = mx,
        .y = my,
        .f = 0,
        .g = 0,
        .parent = NULL
      };
    }
    

  }
 /*
  for (int i = 0; i < total_number_of_loop; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = start.pose.position.x + x_increment * i;
    pose.pose.position.y = start.pose.position.y + y_increment * i;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    // check costmap and check if ha
    unsigned int mx, my;
    if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
      RCLCPP_ERROR(
        node_->get_logger(), "The goal is off the global costmap");
      return global_path;
    }
    // check costmap and check if it clips a layer
    if (costmap_->getCost(mx, my) == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      RCLCPP_ERROR(
        node_->get_logger(), "The goal is in collision");
      return global_path;
    }

    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }*/

  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header.stamp = node_->now();
  goal_pose.header.frame_id = global_frame_;
  global_path.poses.push_back(goal_pose);

  return global_path;
}


}  // namespace astar_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(astar_planner::Astar, nav2_core::GlobalPlanner)
