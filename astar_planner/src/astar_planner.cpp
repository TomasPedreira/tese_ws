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

  Node get_lowest_f_node(
    std::vector<Node> & open_list
  )
  {
    if (open_list.empty()) {
      throw std::runtime_error("open_list is empty!"); // Handle empty case
    }

    auto lowest_it = open_list.begin();
    for (auto it = open_list.begin(); it != open_list.end(); ++it) {
        if (*it < *lowest_it) {  // Assuming `operator<` is defined for Node
            lowest_it = it;
        }
    }

    Node lowest_f_node = *lowest_it;
    

    return lowest_f_node;
  }

  void update_neighbours
  (
    Node current_node, 
    std::vector<Node> & open_list, 
    std::vector<std::vector<bool>> & closed_list,
    std::vector<std::vector<Node>> & node_map,
    std::vector<std::vector<int>> & f_cost_map,
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
      if (f_cost_map[x][y] != -1) {
        if (successor.f < f_cost_map[x][y]) {
          f_cost_map[x][y] = successor.f;
          node_map[x][y] = successor;
        }
      } else {
        node_map[x][y] = successor;
        f_cost_map[x][y] = successor.f;
      }
      auto it_open = std::find(open_list.begin(), open_list.end(), successor);

      if (it_open == open_list.end() && !closed_list[x][y]) {
        open_list.push_back(successor);
      }
      if (x == goal_x && y == goal_y) {
        *path_found = true;
        node_map[x][y] = successor;
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
    std::vector<Node> open_list;
    std::vector<std::vector<bool>> closed_list(costmap_->getSizeInCellsX(), std::vector<bool>(costmap_->getSizeInCellsY(), false));
    std::vector<std::vector<Node>> node_map = std::vector<std::vector<Node>>(costmap_->getSizeInCellsX(), std::vector<Node>(costmap_->getSizeInCellsY()));
    std::vector<std::vector<int>> f_cost_map(costmap_->getSizeInCellsX(), std::vector<int>(costmap_->getSizeInCellsY(), -1));
   

    
    Node start_node;
    start_node.x = start_x;
    start_node.y = start_y;
    start_node.g = 0;
    start_node.h = calculateDist(start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
    start_node.f = start_node.g + start_node.h;
    start_node.yaw = start_orientation;
    start_node.parent = nullptr;

    open_list.push_back(start_node);
    node_map[start_x][start_y] = start_node;

    Node goal_node;
    goal_node.x = goal_x;
    goal_node.y = goal_y;
    goal_node.h = 0;
    std::pair<unsigned int, unsigned int> prev_coord = {start_x, start_y};
    
    // compute a path
    while (!path_found) {
      Node current = get_lowest_f_node(open_list);
      open_list.erase(std::remove(open_list.begin(), open_list.end(), current), open_list.end());
      if (prev_coord.first == current.x || prev_coord.second == current.y) {
        RCLCPP_INFO(
          node_->get_logger(), "REPEATED OH MY MGOD Current node: %d, %d", current.x, current.y);
      }
      prev_coord = {current.x, current.y};

      if (current.f == 9999999) {
        RCLCPP_ERROR(
          node_->get_logger(), "No path found");
        return global_path;
      }
      update_neighbours(current, open_list, closed_list, node_map, f_cost_map,dir, &path_found, goal_x, goal_y, costmap_);
      closed_list[current.x][current.y] = true;
    }
    Node current = node_map[goal_x][goal_y];
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
    RCLCPP_INFO(
      node_->get_logger(), "Path found");
    return global_path;
  }
}  // namespace astar_planner

#include "pluginlib/class_list_macros.hpp"
  PLUGINLIB_EXPORT_CLASS(astar_planner::Astar, nav2_core::GlobalPlanner)