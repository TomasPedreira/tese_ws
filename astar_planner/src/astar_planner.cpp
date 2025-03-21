#include <cmath>
#include <string>
#include <memory>
#include <queue>
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <fstream>
#include "nav2_util/node_utils.hpp"
#include "astar_planner/astar_planner.hpp"
#include "astar_planner/node2d.hpp"
#include "astar_planner/nodeHybrid.hpp"


namespace astar_planner
{


  std::vector<std::vector<nodeHybrid>> read_nodes(string filename, nav2_costmap_2d::Costmap2D * costmap_, rclcpp::Logger logger)
  {
      // open file

      // read file and store in node_map
      // file struture is index x y numofneighbours neighbour1x neighbour1y neighbour2x neighbour2y ... neighbourNx neighbourNy
      // first line is the number of nodes
      // node map is a 2d vector of hybrid nodes indexed by x and y positions
      // x and y is in aboslute coordinates, and need to be converted to costmap coordinates
      std::vector<std::vector<nodeHybrid>> node_map = std::vector<std::vector<nodeHybrid>>(costmap_->getSizeInCellsX(), std::vector<nodeHybrid>(costmap_->getSizeInCellsY()));
      std::ifstream file;
      file.open(filename);
      if (!file.is_open()) {
          return node_map;
      }
      int num_nodes;
      file >> num_nodes;
      // log the number of nodes
      RCLCPP_INFO(
        logger, "Number of nodes: %d", num_nodes);
      for (int i = 0; i < num_nodes; i++) {
          float x, y;
          int index, num_neighbours;
          file >> index >> x >> y >> num_neighbours;
          unsigned int costmap_x, costmap_y;
          std::vector<std::pair<int, int>> neighbours;
          // log the node
          RCLCPP_INFO(
            logger, "Node: %f, %f", x, y);
          if (!costmap_->worldToMap(x, y, costmap_x, costmap_y)) {
              
          }
          for (int j = 0; j < num_neighbours; j++) {
              float neighbour_x, neighbour_y;
              file >> neighbour_x >> neighbour_y;
              unsigned int costmap_neighbour_x, costmap_neighbour_y;
              if (!costmap_->worldToMap(neighbour_x, neighbour_y, costmap_neighbour_x, costmap_neighbour_y)) {
                  
              }
              neighbours.push_back(std::make_pair(costmap_neighbour_x, costmap_neighbour_y));
              // log the neighbour
              RCLCPP_INFO(
                logger, "Neighbour: %f, %f", neighbour_x, neighbour_y);
          }
          // log creating node
          RCLCPP_INFO(logger, "Creating node: %d, %d, %d", i, costmap_x, costmap_y);
          nodeHybrid node = nodeHybrid(costmap_x, costmap_y, 0, 0, 0, nullptr, neighbours);
          // log created node
          RCLCPP_INFO(
            logger, "Created node: %d", i);
          node_map[costmap_x][costmap_y] = node;
          // log finished first node
          RCLCPP_INFO(
            logger, "Finished node: %d", i);

      }
      file.close();
      return node_map;

  }

  
  double calculateDist(double x1, double y1, double x2, double y2)
  {
    return std::hypot(x2 - x1, y2 - y1);
  }
  double calculateOrientation(double x1, double y1, double x2, double y2)
  {
    return std::atan2(y2 - y1, x2 - x1);
  }

  int get_lowest_f_node(
    std::vector<node2d> & open_list
  )
  {
    if (open_list.empty()) {
      return -1;
    }

    int index = -1;
    double min_f = 999999;
    for (size_t i = 0; i < open_list.size(); i++) {
      if (open_list[i].f < min_f) {
        min_f = open_list[i].f;
        index = i;
      }
    }
    

    return index;
  }

  void update_neighbours
  (
    node2d current_node, 
    std::vector<node2d> & open_list, 
    std::vector<std::vector<bool>> & closed_list,
    std::vector<std::vector<node2d>> & node_map,
    std::vector<std::vector<int>> & f_cost_map,
    std::vector<std::pair<int, int>> & dir, 
    bool * path_found,
    node2d goal_node,
    double tolerance,
    nav2_costmap_2d::Costmap2D * costmap_
  )
  {

    unsigned int goal_x = goal_node.x;
    unsigned int goal_y = goal_node.y;
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
      node2d successor = node2d();
      successor.x = x;
      successor.y = y;
      successor.g = current_node.g + calculateDist(current_node.x, current_node.y, x, y);
      successor.h = calculateDist(x, y, goal_node.x, goal_node.y);
      successor.f = successor.g + successor.h;
      successor.yaw = calculateOrientation(current_node.x, current_node.y, x, y);
      successor.parent = std::make_shared<node2d>(current_node);
      if (successor.f < f_cost_map[x][y]){
        node_map[x][y] = successor;
        f_cost_map[x][y] = successor.f;
      }
        
      bool found = false;
      for (size_t i = 0; i < open_list.size(); i++) {
        if (open_list[i].x == x && open_list[i].y == y) {
          found = true;
          break;
        }
      }

      if (!found && !closed_list[x][y]) {
        open_list.push_back(successor);
      }
      if (calculateDist(x, y, goal_x, goal_y) <=  tolerance) {
        *path_found = true;
        node_map[x][y] = successor;
        if (x != goal_node.x || y != goal_node.y) {
          goal_node.h = 0;
          goal_node.g = successor.g + calculateDist(x, y, goal_x, goal_y);
          goal_node.f = goal_node.g + goal_node.h;
          goal_node.parent = std::make_shared<node2d>(successor);
          node_map[goal_x][goal_y] = goal_node;
        }
        RCLCPP_INFO(
          rclcpp::get_logger("Vitoria"), "Path found");
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
    // print every dir
    // for (size_t i = 0; i < dir.size(); i++) {
    //   RCLCPP_INFO(
    //     rclcpp::get_logger("rclcpp"), "Direction: %d, %d", dir[i].first, dir[i].second);
    // }

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
    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".tolerance", rclcpp::ParameterValue(1.0));
    node_->get_parameter(name_ + ".tolerance", tolerance_);
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

    // double start_orientation = tf2::getYaw(start.pose.orientation); 

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

    
    voronoi_nodes_ = read_nodes("/home/tomas/tt_ws/src/tese_ws/astar_planner/map/voronoi_nodes.txt", costmap_, node_->get_logger());
    // for (size_t i = 0; i < voronoi_nodes_.size(); i++) {
    //   for (size_t j = 0; j < voronoi_nodes_[i].size(); j++) {
    //       RCLCPP_INFO(
    //         node_->get_logger(), "Node: %d, %d", voronoi_nodes_[i][j].x, voronoi_nodes_[i][j].y);
        
    //   }
    // }
    auto dir = calculatePossibleDirectionsx(tolerance_);
    std::vector<node2d> open_list;
    std::vector<std::vector<bool>> closed_list(costmap_->getSizeInCellsX(), std::vector<bool>(costmap_->getSizeInCellsY(), false));
    std::vector<std::vector<node2d>> node_map = std::vector<std::vector<node2d>>(costmap_->getSizeInCellsX(), std::vector<node2d>(costmap_->getSizeInCellsY()));
    std::vector<std::vector<int>> f_cost_map(costmap_->getSizeInCellsX(), std::vector<int>(costmap_->getSizeInCellsY(), -1));
   

    
    node2d start_node = node2d();
    start_node.x = start_x;
    start_node.y = start_y;
    start_node.g = 0;
    start_node.h = calculateDist(start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
    start_node.f = start_node.g + start_node.h;
    start_node.parent = nullptr;

    open_list.push_back(start_node);
    node_map[start_x][start_y] = start_node;

    node2d goal_node = node2d();
    goal_node.x = goal_x;
    goal_node.y = goal_y;
    goal_node.h = 0;
    goal_node.yaw = tf2::getYaw(goal.pose.orientation);
    std::pair<unsigned int, unsigned int> prev_coord = {start_x, start_y};
    
    // get current tiimestamp
    auto start_time = std::chrono::high_resolution_clock::now();
    // compute a path
    while (!path_found) {

      int current_index = get_lowest_f_node(open_list);
      if (current_index == -1) {
        RCLCPP_ERROR(
          node_->get_logger(), "No path found");
        return global_path;
      }
      node2d current = open_list[current_index];  
      open_list.erase(open_list.begin() + current_index);
      closed_list[current.x][current.y] = true;
      

      if (prev_coord.first == current.x && prev_coord.second == current.y && prev_coord.first != start_x && prev_coord.second != start_y) {
        RCLCPP_INFO(
          node_->get_logger(), "REPEATED OH MY MGOD Current node: %d, %d", current.x, current.y);
      }
      prev_coord = {current.x, current.y};
      
      
      update_neighbours(current, open_list, closed_list, node_map, f_cost_map,dir, &path_found, goal_node, tolerance_,costmap_);
      
    }
    // print the time taken to compute the path
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO(
      node_->get_logger(), "Time taken to compute path: %ld ms", duration.count());
    node2d current = node_map[goal_x][goal_y];
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