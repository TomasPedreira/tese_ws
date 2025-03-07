#include <cmath>
#include <string>
#include <memory>
#include <queue>
#include <vector>
#include <unordered_set>
#include <algorithm>
#include "nav2_util/node_utils.hpp"
#include "astar_planner/astar_planner.hpp"

namespace astar_planner
{

// Node structure for A* algorithm
struct node_ {
  unsigned int x, y;  // Changed from int to unsigned int to match costmap types
  double f, g, h;
  std::shared_ptr<Node> parent;

  // For priority queue comparison
  bool operator>(const Node & other) const
  {
    return f > other.f;
  }

  // For hash table
  bool operator==(const Node & other) const
  {
    return x == other.x && y == other.y;
  }
};

// Hash function for Node to use in unordered sets
struct NodeHash {
  std::size_t operator()(const Node & node) const
  {
    return std::hash<unsigned int>()(node.x) ^ std::hash<unsigned int>()(node.y);
  }
};

double calculateDist(double x1, double y1, double x2, double y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}
double calculateOrientation(double x1, double y1, double x2, double y2)
{
  return std::atan2(y2 - y1, x2 - x1);
}

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
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  
  // Additional parameters for A*
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_diagonal", rclcpp::ParameterValue(true));
  node_->get_parameter(name_ + ".allow_diagonal", allow_diagonal_);
  
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".heuristic_weight", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".heuristic_weight", heuristic_weight_);
}

void Astar::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type AstarPlanner",
    name_.c_str());
}

void Astar::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type AstarPlanner",
    name_.c_str());
}

void Astar::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type AstarPlanner",
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
      node_->get_logger(), "Planner will only accept start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only accept goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  unsigned int start_x, start_y, goal_x, goal_y;
  if (!costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y, start_x, start_y))
  {
    RCLCPP_ERROR(
      node_->get_logger(), "The start position is off the global costmap");
    return global_path;
  }

  if (!costmap_->worldToMap(
      goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
  {
    RCLCPP_ERROR(
      node_->get_logger(), "The goal position is off the global costmap");
    return global_path;
  }

  // Check if start or goal is in collision
  if (costmap_->getCost(start_x, start_y) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
    RCLCPP_ERROR(
      node_->get_logger(), "The start position is in collision");
    return global_path;
  }

  if (costmap_->getCost(goal_x, goal_y) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
    RCLCPP_ERROR(
      node_->get_logger(), "The goal position is in collision");
    return global_path;
  }

  // A* algorithm
  // Open list - priority queue (lowest f value first)
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
  // Closed set - nodes already evaluated
  std::unordered_set<Node, NodeHash> closed_set;
  
  // Initialize the start node
  auto start_node = std::make_shared<Node>();
  start_node->x = start_x;
  start_node->y = start_y;
  start_node->g = 0;
  start_node->h = heuristic_weight_ * calculateDist(
    start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);
  start_node->f = start_node->g + start_node->h;
  start_node->parent = nullptr;
  
  // Add start node to open list
  open_list.push(*start_node);
  
  // Maps to store the best g value for each position
  std::unordered_map<unsigned int, std::unordered_map<unsigned int, double>> best_g_cost;
  best_g_cost[start_x][start_y] = 0;
  
  // Maps to store the node pointers for path reconstruction
  std::unordered_map<unsigned int, std::unordered_map<unsigned int, std::shared_ptr<Node>>> node_map;
  node_map[start_x][start_y] = start_node;
  
  // Direction vectors for 8-connected grid (including diagonals)
  // const int dx[8] = {-1, 0, 1, 0, -1, -1, 1, 1};
  // const int dy[8] = {0, 1, 0, -1, -1, 1, -1, 1};
  const int dx[8] = {-3, 0, 3, 0, -3, -3, 3, 3};
  const int dy[8] = {0, 3, 0, -3, -3, 3, -3, 3};

  // int dir_limit = allow_diagonal_ ? 8 : 4;  // Use 8 directions if diagonals allowed, else 4
  int dir_limit = sizeof(dx)/sizeof(const int);
  RCLCPP_INFO(node_->get_logger(), "dir_limit: %d", dir_limit);
  RCLCPP_INFO(node_->get_logger(), "Res: %f", costmap_->getResolution());
  RCLCPP_INFO(node_->get_logger(), "Distance to goal: %f", calculateDist(start_node->x, start_node->y, goal_x, goal_y));
  
  bool path_found = false;
  const double tolerance = 3.0;
  int count = 0;
  while (!open_list.empty()) {
    // Get the node with the lowest f value
    Node current_node = open_list.top();
    open_list.pop();
    
    // If we've reached the goal
    if(calculateDist(current_node.x, current_node.y, goal_x, goal_y) <= tolerance) {
      RCLCPP_INFO(node_->get_logger(), "Distance to final goal: %f", calculateDist(current_node.x, current_node.y, goal_x, goal_y));
      RCLCPP_INFO(node_->get_logger(), "Count: %d", count);
      path_found = true;
      // add goal to node map
      auto goal_node = std::make_shared<Node>();
      goal_node->x = goal_x;
      goal_node->y = goal_y;
      goal_node->g = current_node.g;
      goal_node->h = 0;
      goal_node->f = goal_node->g + goal_node->h;
      goal_node->parent = node_map[current_node.x][current_node.y];
      node_map[goal_x][goal_y] = goal_node;
      break;
    }
    
    // if (current_node.x == goal_x && current_node.y == goal_y) {
    //   path_found = true;
    //   break;
    // }
    
    // Skip if this node has already been evaluated
    if (closed_set.find(current_node) != closed_set.end()) {
      continue;
    }
    
    // Add to closed set
    closed_set.insert(current_node);
    
    // Check all neighbors
    
    for (int i = 0; i < dir_limit; i++) {
      // Use signed int for calculations to avoid underflow
      int new_x_signed = static_cast<int>(current_node.x) + dx[i];
      int new_y_signed = static_cast<int>(current_node.y) + dy[i];
      
      // Skip if outside map bounds
      if (new_x_signed < 0 || new_x_signed >= static_cast<int>(costmap_->getSizeInCellsX()) || 
          new_y_signed < 0 || new_y_signed >= static_cast<int>(costmap_->getSizeInCellsY())) {
        continue;
      }
      
      // Convert back to unsigned for costmap access
      unsigned int new_x = static_cast<unsigned int>(new_x_signed);
      unsigned int new_y = static_cast<unsigned int>(new_y_signed);
      
      // Skip if in collision or already in closed set
      Node neighbor = {new_x, new_y, 0, 0, 0, nullptr};
      if (costmap_->getCost(new_x, new_y) >= nav2_costmap_2d::LETHAL_OBSTACLE ||
          closed_set.find(neighbor) != closed_set.end()) {
        continue;
      }
      
      // Calculate cost to this neighbor as distance
      double movement_cost = calculateDist(dx[i], dy[i], 0, 0);
      // double movement_cost = (i < 4) ? 1.0 : 1.414; 
      
      // Cost also depends on the cell's cost value
      unsigned char cost = costmap_->getCost(new_x, new_y);
      if (cost != nav2_costmap_2d::FREE_SPACE) {
        // Scale movement cost based on cell cost
        double cost_penalty = cost / 255.0 * 10.0;  // Scale from 0 to 10
        movement_cost += cost_penalty;
      }
      
      double tentative_g = node_map[current_node.x][current_node.y]->g + movement_cost;
      
      // Only proceed if this path is better than any previous one to this cell
      if (best_g_cost.find(new_x) == best_g_cost.end() || 
          best_g_cost[new_x].find(new_y) == best_g_cost[new_x].end() ||
          tentative_g < best_g_cost[new_x][new_y]) {
        
        // Convert to world coordinates for heuristic calculation
        double wx, wy;
        costmap_->mapToWorld(new_x, new_y, wx, wy);
        
        // Create a new node
        auto new_node = std::make_shared<Node>();
        new_node->x = new_x;
        new_node->y = new_y;
        new_node->g = tentative_g;
        new_node->h = heuristic_weight_ * calculateDist(wx, wy, goal.pose.position.x, goal.pose.position.y);
        new_node->f = new_node->g + new_node->h;
        new_node->parent = node_map[current_node.x][current_node.y];
        
        // Update best path to this position
        best_g_cost[new_x][new_y] = tentative_g;
        node_map[new_x][new_y] = new_node;
        
        // Add to open list
        open_list.push(*new_node);
      }
    }
  }
  
  // Reconstruct path if goal was reached
  if (path_found) {
    std::vector<geometry_msgs::msg::PoseStamped> path;
    std::shared_ptr<Node> current = node_map[goal_x][goal_y];
    // push goal pose
    geometry_msgs::msg::PoseStamped goal_pose = goal;
    goal_pose.header.stamp = node_->now();
    goal_pose.header.frame_id = global_frame_;
    path.push_back(goal_pose);

    
    while (current != nullptr) {
      geometry_msgs::msg::PoseStamped pose;
      double wx, wy;
      costmap_->mapToWorld(current->x, current->y, wx, wy);
      
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
      
      path.push_back(pose);
      current = current->parent;
    }
    
    // Path needs to be reversed since we backtracked from goal to start
    std::reverse(path.begin(), path.end());
    global_path.poses = path;
    
    
    RCLCPP_INFO(
      node_->get_logger(), "A* planner found a path with %zu points", 
      global_path.poses.size());
  } else {
    RCLCPP_WARN(node_->get_logger(), "A* planner could not find a path");
  }
  
  return global_path;
}

}  // namespace astar_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(astar_planner::Astar, nav2_core::GlobalPlanner)