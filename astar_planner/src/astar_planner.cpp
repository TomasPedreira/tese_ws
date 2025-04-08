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
#include "astar_planner/dubins.hpp"
// #include "astar_planner/voronoi.hpp"


namespace astar_planner
{

  double euclidean_distance(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) {
    return std::sqrt(std::pow(static_cast<double>(x1) - x2, 2) + std::pow(static_cast<double>(y1) - y2, 2));
  }

  std::vector<nodeHybrid> read_nodes(const std::string& filename, nav2_costmap_2d::Costmap2D* costmap_) {
    std::vector<nodeHybrid> voronoi_nodes;
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return voronoi_nodes;
    }

    int num_nodes;
    file >> num_nodes;
    if (file.fail()) {
        std::cerr << "Error: Failed to read number of nodes" << std::endl;
        return voronoi_nodes;
    }
    file.ignore(); // Ignore newline after reading num_nodes

    std::cout << "Reading " << num_nodes << " nodes from file" << std::endl;

    for (int i = 0; i < num_nodes; ++i) {
        std::string line;
        if (!std::getline(file, line)) {
            std::cerr << "Error: Unexpected end of file at node " << i << std::endl;
            break;
        }

        std::istringstream iss(line);
        int id, num_neighbors;
        unsigned int x, y;

        if (!(iss >> id >> y >> x >> num_neighbors)) {
            std::cerr << "Error reading node " << i << " from line: " << line << std::endl;
            continue;
        }

        std::vector<int> neighbors;
        for (int j = 0; j < num_neighbors; ++j) {
            int neighbor_id;
            if (!(iss >> neighbor_id)) {
                std::cerr << "Error reading neighbor " << j << " for node " << id << " from line: " << line << std::endl;
                break;
            }
            neighbors.push_back(neighbor_id);
        }

        if (x > costmap_->getSizeInCellsX() || y > costmap_->getSizeInCellsY()) {
            std::cerr << "Node " << id << " is outside costmap bounds" << costmap_->getSizeInCellsX() << costmap_->getSizeInCellsY() << std::endl;
            voronoi_nodes.emplace_back(id, true, 999999, 99999, nullptr, neighbors);
            continue;
        }

        voronoi_nodes.emplace_back(id, false, x, y, nullptr, neighbors);
    }

    std::cout << "Successfully loaded " << voronoi_nodes.size() << " nodes" << std::endl;
    return voronoi_nodes;
  }

  std::vector<int> get_neighbours(const std::vector<nodeHybrid>& voronoi_nodes, 
    const nodeHybrid& start_node, 
    nav2_costmap_2d::Costmap2D* costmap_, 
    int k) 
  {  
    std::vector<std::pair<int, double>> nearest_nodes; // (Node ID, Distance)
    for (const auto& node : voronoi_nodes) {
      double distance = euclidean_distance(start_node.x, start_node.y, node.x, node.y);
      if (!node.is_outside){
        unsigned char cost = costmap_->getCost(node.x, node.y);
        if (cost < nav2_costmap_2d::LETHAL_OBSTACLE) {  
          nearest_nodes.push_back({node.id, distance});
        }
      }
    }
    // Sort nodes by distance (ascending)
    std::sort(nearest_nodes.begin(), nearest_nodes.end(), [](const auto& a, const auto& b) {
      return a.second < b.second;
    });

    // Collect the `k` nearest nodes
    std::vector<int> closest_node_ids;
    for (int i = 0; i < std::min(k, static_cast<int>(nearest_nodes.size())); i++) {
      closest_node_ids.push_back(nearest_nodes[i].first);
    }

    return closest_node_ids;
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
    std::vector<nodeHybrid> & open_list
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
    nodeHybrid current_node, 
    std::vector<nodeHybrid> voronoi_nodes,
    std::vector<nodeHybrid> & open_list, 
    std::vector<std::vector<bool>> & closed_list,
    std::vector<std::vector<nodeHybrid>> & node_map,
    std::vector<std::vector<double>> & f_cost_map,
    bool * path_found,
    nodeHybrid goal_node,
    double tolerance,
    nav2_costmap_2d::Costmap2D * costmap_
  )
  {
    unsigned int goal_x = goal_node.x;
    unsigned int goal_y = goal_node.y;
    for (size_t i = 0; i < current_node.neighbours.size(); i++) {
      nodeHybrid successor = voronoi_nodes[current_node.neighbours[i]];
      unsigned int x = successor.x;
      unsigned int y = successor.y;
      if (x >= costmap_->getSizeInCellsX() || y >= costmap_->getSizeInCellsY()) {
        continue;
      }
      if (costmap_->getCost(x, y) != 0) {
        continue;
      }
      
      successor.g = current_node.g + calculateDist(current_node.x, current_node.y, x, y);
      successor.h = calculateDist(x, y, goal_node.x, goal_node.y);
      successor.f = successor.g + successor.h;
      successor.yaw = calculateOrientation(current_node.x, current_node.y, x, y);
      successor.parent = std::make_shared<nodeHybrid>(current_node);
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
          goal_node.parent = std::make_shared<nodeHybrid>(successor);
          node_map[goal_x][goal_y] = goal_node;
        }
        RCLCPP_INFO(
          rclcpp::get_logger("Vitoria"), "Path found");
        return;
      }
    }
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
      node_, name_ + ".tolerance", rclcpp::ParameterValue(1.0));
    node_->get_parameter(name_ + ".tolerance", tolerance_);

    // Dubins parameters
    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".turning_radius", rclcpp::ParameterValue(1.0));
    node_->get_parameter(name_ + ".turning_radius", turning_radius_);
    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".dubins_tolerance", rclcpp::ParameterValue(0.2));
    node_->get_parameter(name_ + ".dubins_tolerance", dubins_tolerance_);

    
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
    voronoi_nodes_ = read_nodes(
      "/home/tomas/tt_ws/src/tese_ws/astar_planner/map/voronoi_nodes.txt", 
      costmap_
    );
  }

  void Astar::deactivate()
  {
    RCLCPP_INFO(
      node_->get_logger(), "Deactivating plugin %s of type AstarPlanner",
      name_.c_str());
  }

  std::vector<nodeHybrid> compute_subgoals (
    const std::vector<nodeHybrid> & voronoi_nodes,
    const nodeHybrid & start_node,
    const nodeHybrid & goal_node,
    nav2_costmap_2d::Costmap2D* costmap_,
    double tolerance
  )
  {

  
    std::vector<nodeHybrid> open_list;
    std::vector<nodeHybrid> sub_goals;
    std::vector<std::vector<bool>> closed_list(costmap_->getSizeInCellsX(), std::vector<bool>(costmap_->getSizeInCellsY(), false));
    std::vector<std::vector<nodeHybrid>> node_map = std::vector<std::vector<nodeHybrid>>(costmap_->getSizeInCellsX(), std::vector<nodeHybrid>(costmap_->getSizeInCellsY()));
    std::vector<std::vector<double>> f_cost_map(costmap_->getSizeInCellsX(), std::vector<double>(costmap_->getSizeInCellsY(), -1));
    
    unsigned int start_x = start_node.x;
    unsigned int start_y = start_node.y;
    unsigned int goal_x = goal_node.x;
    unsigned int goal_y = goal_node.y;

    open_list.push_back(start_node);
    node_map[start_x][start_y] = start_node;
        
    // get current tiimestamp
    auto start_time = std::chrono::high_resolution_clock::now();
    // compute a path
    int iter = 0;
    bool path_found = false;
    while (!path_found) {
      cout << "Iteration: " << iter << endl;
      iter++;
    
      int current_index = get_lowest_f_node(open_list);
      if (current_index == -1) {
        return sub_goals;
      }
      nodeHybrid current = open_list[current_index];  
      open_list.erase(open_list.begin() + current_index);
      cout << "Current node: " << current.id << " x: " << current.x << " y: " << current.y << endl;
      closed_list[current.x][current.y] = true;
            
      update_neighbours(current, voronoi_nodes,open_list, closed_list, node_map, f_cost_map, &path_found, goal_node, tolerance,costmap_);
      
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    nodeHybrid current_node = node_map[goal_x][goal_y];
    nodeHybrid prev_node = current_node;
    sub_goals.insert(sub_goals.begin(), current_node); 
    current_node = *current_node.parent;
    while (current_node.parent != nullptr) {
      current_node.yaw = calculateOrientation(current_node.x, current_node.y, prev_node.x, prev_node.y);
      sub_goals.insert(sub_goals.begin(), current_node);
      prev_node = current_node;
      current_node = *current_node.parent;
    }
    sub_goals.insert(sub_goals.begin(), current_node);
    

    return sub_goals;
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

    

    nodeHybrid start_node = nodeHybrid();
    start_node.x = start_x;
    start_node.y = start_y;
    start_node.g = 0;
    start_node.h = calculateDist(start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
    start_node.f = start_node.g + start_node.h;
    start_node.yaw = tf2::getYaw(start.pose.orientation);
    start_node.parent = nullptr;
    

    start_node.neighbours = get_neighbours(voronoi_nodes_, start_node, costmap_, 5);
    std::vector<nodeHybrid> sub_nodes;
   
    
    nodeHybrid goal_node = nodeHybrid();
    goal_node.x = goal_x;
    goal_node.y = goal_y;
    goal_node.h = 0;
    goal_node.yaw = tf2::getYaw(goal.pose.orientation);
    cout << "Start node: " << start_node.x << " " << start_node.y << endl;
    cout << "Goal node: " << goal_node.x << " " << goal_node.y << endl;
    
   
    std::vector<nodeHybrid> subgoals = compute_subgoals(voronoi_nodes_, start_node, goal_node, costmap_, tolerance_);
    std::vector<nodeHybrid> dubins_path = create_dubins_path(costmap_ ,start_node, goal_node, turning_radius_, dubins_tolerance_);
    std::vector<nodeHybrid> path_to_consider = subgoals;
    


    if (path_to_consider.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Dubins path is empty, falling back to straight line");
      path_to_consider.push_back(start_node);
      path_to_consider.push_back(goal_node);
    }
    for (size_t i = 0; i < path_to_consider.size(); i++) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
      pose.pose.position.x = costmap_->getOriginX() + path_to_consider[i].x * costmap_->getResolution();
      pose.pose.position.y = costmap_->getOriginY() + path_to_consider[i].y * costmap_->getResolution();
      pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), path_to_consider[i].yaw));
      // cout << "Pose: " << pose.pose.position.x << " " << pose.pose.position.y << endl;
      // global_path.poses.insert(global_path.poses.begin(), pose);
      global_path.poses.push_back(pose);     
    }
    return global_path;
  }
}  // namespace astar_planner

#include "pluginlib/class_list_macros.hpp"
  PLUGINLIB_EXPORT_CLASS(astar_planner::Astar, nav2_core::GlobalPlanner)