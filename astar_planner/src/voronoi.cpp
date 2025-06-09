#include <memory>
#include <vector>
#include <fstream>
#include "astar_planner/node2d.hpp"
#include "astar_planner/nodeHybrid.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include "astar_planner/voronoi.hpp"
#include "astar_planner/auxiliary_functions.hpp"
#include <iostream>


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
        if (cost < 254) {  
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

  void update_neighbours
  (
    nodeHybrid current_node, 
    std::vector<nodeHybrid> voronoi_nodes,
    std::vector<nodeHybrid> & open_list, 
    std::vector<std::vector<bool>> & closed_list,
    std::vector<std::vector<nodeHybrid>> & node_map,
    std::vector<std::vector<double>> & f_cost_map,
    nodeHybrid goal_node,
    nav2_costmap_2d::Costmap2D * costmap_
  )
  {
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
      successor.h = calculateDist(x, y, goal_node.x, goal_node.y) ;
      successor.f = successor.g + successor.h;
      successor.parent = std::make_shared<nodeHybrid>(current_node);
      if (current_node.parent != nullptr){
        double projected_parent_yaw = calculateOrientation(current_node.x, current_node.y, successor.x, successor.y);
        double yaw_diff = projected_parent_yaw - current_node.parent->yaw;
        while (yaw_diff > M_PI){
          yaw_diff -= 2*M_PI;
        }
        while (yaw_diff < -M_PI){
          yaw_diff += 2*M_PI;
        }
        if (std::abs(yaw_diff) > M_PI*0.9){
          continue;
        }
      }
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
    }
  }

  std::vector<nodeHybrid> compute_subgoals (
    const std::vector<nodeHybrid> & voronoi_nodes,
    const nodeHybrid & start_node,
    nodeHybrid & goal_node,
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
    node_map[start_x][start_y].is_voronoi = true;  // Set node type at creation
        
    // get current tiimestamp
    auto start_time = std::chrono::high_resolution_clock::now();
    // compute a path
    int iter = 0;
    bool path_found = false;
    nodeHybrid current;
    while (!path_found) {
      iter++;
    
      int current_index = get_lowest_f_node(open_list);
      if (current_index == -1) {
        cout << "Open list empty, returning empty subgoals" << endl;
        return sub_goals;
      }
      current = open_list[current_index];  
      if (current.parent != nullptr){
        current.parent->yaw = calculateOrientation(current.parent->x, current.parent->y, current.x, current.y);
      }
      open_list.erase(open_list.begin() + current_index);

      
      
      closed_list[current.x][current.y] = true;
      if (calculateDist(current.x, current.y, goal_x, goal_y) <=  tolerance) {
        path_found = true;
        double projected_yaw = calculateOrientation(current.x, current.y, goal_node.x, goal_node.y);
        double yaw_diff = projected_yaw - goal_node.yaw;
        while (yaw_diff > M_PI){
          yaw_diff -= 2*M_PI;
        }
        while (yaw_diff < -M_PI){
          yaw_diff += 2*M_PI;
        }
        if (std::abs(yaw_diff) > M_PI/2){
          cout << "skipping node because yaw_diff_forward at goal: " << yaw_diff << endl;
          path_found = false;
          continue;
        }
        current.yaw = projected_yaw;
        goal_node.parent = std::make_shared<nodeHybrid>(current);
        node_map[goal_x][goal_y] = goal_node;
        break;
      }
            
      update_neighbours(current, voronoi_nodes,open_list, closed_list, node_map, f_cost_map, goal_node,costmap_);

    }
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    nodeHybrid current_node = node_map[goal_x][goal_y];
    nodeHybrid prev_node = current_node;
    sub_goals.insert(sub_goals.begin(), current_node); 
    current_node = *current_node.parent;
    while (current_node.parent != nullptr) {
      current_node.is_voronoi = true;
      // current_node.yaw = calculateOrientation(current_node.x, current_node.y, prev_node.x, prev_node.y);
      sub_goals.insert(sub_goals.begin(), current_node);
      prev_node = current_node;
      current_node = *current_node.parent;
    }
    current_node.is_voronoi = true;
    sub_goals.insert(sub_goals.begin(), current_node);
    

    return sub_goals;
  }
