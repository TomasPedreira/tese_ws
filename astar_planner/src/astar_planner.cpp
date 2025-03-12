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
  struct Node {
    unsigned int x, y;  // Changed from int to unsigned int to match costmap types
    double f, g, h;
    double yaw;
    std::shared_ptr<Node> parent;

    // For priority queue comparison
    bool operator>(const Node & other) const
    {
      return f > other.f;
    }
    bool operator<(const Node & other) const
    {
      return f < other.f;
    }

    // For hash table
    bool operator==(const Node & other) const
    {
      return x == other.x && y == other.y && yaw == other.yaw;
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

  bool has_been_checked(
    std::vector<std::pair<unsigned int, unsigned int>> & checked_coords,
    unsigned int x,
    unsigned int y
  )
  {
    for (size_t i = 0; i < checked_coords.size(); i++) {
      if (x == checked_coords[i].first && y == checked_coords[i].second) {
        return true;
      }
    }
    return false;
  }

  bool has_non_checked_neighbours(
    std::vector<std::pair<unsigned int, unsigned int>> & checked_coords,
    std::vector<std::pair<int, int>> & dir,
    Node current_node,
    nav2_costmap_2d::Costmap2D * costmap_
  )
  {
    for (size_t i = 0; i < dir.size(); i++) {
      if (current_node.x == 0 && dir[i].first < 0) {
        continue;
      }
      if (current_node.y == 0 && dir[i].second < 0) {
        continue;
      }
      unsigned int x = current_node.x + dir[i].first;
      unsigned int y = current_node.y + dir[i].second;
      if (x >= costmap_->getSizeInCellsX() || y >= costmap_->getSizeInCellsY()) {
        continue;
      }
      if (costmap_->getCost(x, y) != 0) {
        continue;
      }
      if (!has_been_checked(checked_coords, x, y)) {
        return true;
      }
    }
    return false;
  }
    

  Node get_lowest_f_node(
    std::vector<std::vector<Node>> & checked_nodes,
    std::vector<std::pair<unsigned int, unsigned int>> & checked_coords,
    std::vector<std::pair<int, int>> & dir,
    nav2_costmap_2d::Costmap2D * costmap_
  )
  {
    Node lowest_f_node;
    lowest_f_node.f = 9999999;
    for (size_t i = 0; i < checked_coords.size(); i++) {
      if (has_non_checked_neighbours(checked_coords, dir,checked_nodes[checked_coords[i].first][checked_coords[i].second], costmap_)) {
        if (checked_nodes[checked_coords[i].first][checked_coords[i].second] < lowest_f_node) {
          lowest_f_node = checked_nodes[checked_coords[i].first][checked_coords[i].second];
        }
      }
    }
    return lowest_f_node;
  }

  void update_neighbours
  (
    Node current_node, 
    std::vector<std::vector<Node>> & checked_nodes, 
    std::vector<std::pair<unsigned int, unsigned int>> & checked_coords, 
    std::vector<std::pair<int, int>> & dir, 
    bool * path_found,
    unsigned int goal_x,
    unsigned int goal_y,
    nav2_costmap_2d::Costmap2D * costmap_
  )
  {
    for (size_t i = 0; i < dir.size(); i++) {
      if (current_node.x == 0 && dir[i].first < 0) {
        continue;
      }
      if (current_node.y == 0 && dir[i].second < 0) {
        continue;
      }
      unsigned int x = current_node.x + dir[i].first;
      unsigned int y = current_node.y + dir[i].second;
      if (x >= costmap_->getSizeInCellsX() || y >= costmap_->getSizeInCellsY()) {
        continue;
      }
      if (costmap_->getCost(x, y) != 0) {
        continue;
      }
      Node successor;
      successor.x = x;
      successor.y = y;
      successor.g = current_node.g + calculateDist(current_node.x, current_node.y, x, y);
      successor.h = calculateDist(x, y, goal_x, goal_y);
      successor.f = successor.g + successor.h;
      successor.yaw = calculateOrientation(current_node.x, current_node.y, x, y);
      successor.parent = std::make_shared<Node>(current_node);
      if (has_been_checked(checked_coords, x, y)) {
        if (successor < checked_nodes[x][y]) {
          checked_nodes[x][y] = successor;
        }
      } else {
        checked_nodes[x][y] = successor;
        checked_coords.push_back({x, y});
      }
      if (x == goal_x && y == goal_y) {
        *path_found = true;
        return;
      }
    }
  }




  std::vector<std::pair<int, int>>calculatePossibleDirectionsx(int scale)
  {
    std::vector<std::pair<int, int>> dir;

    for (int i = -scale; i <= scale; i++) {
        for (int j = -scale; j <= scale; j++) {
            if (i == 0 && j == 0) {
                continue;  // Skip the (0, 0) direction
            }
            dir.push_back({i, j});
        }
    }

    return dir;  // Return the computed directions
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

    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".max_steer", rclcpp::ParameterValue(90.0));
    node_->get_parameter(name_ + ".max_steer", max_steer_);
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
    const geometry_msgs::msg::PoseStamped & goal
  )
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

    double start_orientation = tf2::getYaw(start.pose.orientation); 

    unsigned int start_x, start_y, goal_x, goal_y;
    if (!costmap_->worldToMap(
        start.pose.position.x, start.pose.position.y, start_x, start_y)
    )
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

    bool path_found = false;
    // create list of nodes

    auto dir = calculatePossibleDirectionsx(1);
    std::vector<std::vector<Node>> checked_nodes(costmap_->getSizeInCellsX(), std::vector<Node>(costmap_->getSizeInCellsY()));
    // list of pairs of int checked
    std::vector<std::pair<unsigned int, unsigned int>> checked_coords;

    
    Node start_node;
    start_node.x = start_x;
    start_node.y = start_y;
    start_node.g = 0;
    start_node.h = calculateDist(start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
    start_node.f = start_node.g + start_node.h;
    start_node.yaw = start_orientation;
    start_node.parent = nullptr;
    checked_nodes[start_x][start_y] = start_node;
    checked_coords.push_back({start_x, start_y});
    update_neighbours(start_node, checked_nodes, checked_coords, dir, &path_found, goal_x, goal_y, costmap_);

    Node goal_node;
    goal_node.x = goal_x;
    goal_node.y = goal_y;
    goal_node.h = 0;

    

    while (!path_found) {
      Node current = get_lowest_f_node(checked_nodes, checked_coords, dir, costmap_);
      // log curent node position and statrting node position
      RCLCPP_INFO(node_->get_logger(), "Current node position: %d, %d, starting psition: %d, %d", current.x, current.y, start_x, start_y);

      if (current.f == 9999999) {
        RCLCPP_ERROR(
          node_->get_logger(), "No path found");
        return global_path;
      }
      update_neighbours(current, checked_nodes, checked_coords, dir, &path_found, goal_x, goal_y, costmap_);
    }

    Node current = checked_nodes[goal_x][goal_y];
    while (current.parent != nullptr) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
      pose.pose.position.x = costmap_->getOriginX() + current.x * costmap_->getResolution();
      pose.pose.position.y = costmap_->getOriginY() + current.y * costmap_->getResolution();
      pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), current.yaw));
      global_path.poses.insert(global_path.poses.begin(), pose);
      current = *current.parent;
    
    
      
    }
    return global_path;
  }
}  // namespace astar_planner

#include "pluginlib/class_list_macros.hpp"
  PLUGINLIB_EXPORT_CLASS(astar_planner::Astar, nav2_core::GlobalPlanner)