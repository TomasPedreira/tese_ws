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
#include "astar_planner/voronoi.hpp"
#include "astar_planner/auxiliary_functions.hpp"

#define DEBUG 0


namespace astar_planner
{
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


    // Hybrid astar parameters
    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".num_directions", rclcpp::ParameterValue(5));
    node_->get_parameter(name_ + ".num_directions", num_directions_);
    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".allow_reverse", rclcpp::ParameterValue(false));
    node_->get_parameter(name_ + ".allow_reverse", allow_reverse_);
    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".max_angle", rclcpp::ParameterValue(M_PI / 4));
    node_->get_parameter(name_ + ".max_angle", max_angle_);
    double step = max_angle_ / num_directions_;
    
    for (int i = -num_directions_ + 1; i < num_directions_; i++) {
      double angle = step * i;
      directions_.push_back(angle);
      
    }
      
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
    
    nodeHybrid goal_node = nodeHybrid();
    goal_node.x = goal_x;
    goal_node.y = goal_y;
    goal_node.h = 0;
    goal_node.yaw = tf2::getYaw(goal.pose.orientation);
    cout << "Start node: " << start_node.x << " " << start_node.y << endl;
    cout << "Goal node: " << goal_node.x << " " << goal_node.y << endl;
    
   
    std::vector<nodeHybrid> subgoals = compute_subgoals(voronoi_nodes_, start_node, goal_node, costmap_, tolerance_);
    // std::vector<nodeHybrid> dubins_path = create_dubins_path(costmap_ ,start_node, goal_node, turning_radius_, dubins_tolerance_);
    std::vector<nodeHybrid> path_to_consider;
    nodeHybrid current_node = start_node;
    nodeHybrid last_subgoal = start_node;
    std::vector<nodeHybrid> open_list;
    open_list.push_back(current_node);

    bool dubins_found = false;
    bool goal_found = false;
    while (!open_list.empty() && !goal_found) {
      cout << "Open list size: " << open_list.size() << endl;
      current_node = open_list[get_lowest_f_node(open_list)];
      open_list.erase(open_list.begin() + get_lowest_f_node(open_list));
      dubins_found = false;
      for (int i = subgoals.size()-1; i > 0; i--) {
        nodeHybrid subgoal = subgoals[i];
        if (subgoal.x == last_subgoal.x && subgoal.y == last_subgoal.y) {
          cout << "Mustache, checking current node" << endl;
          cout << "path wasnt found" << endl;
          break;
        }
        std::vector<nodeHybrid> dubins_path = create_dubins_path(costmap_, current_node, subgoal, turning_radius_, dubins_tolerance_);
        if (!dubins_check_colision(dubins_path, costmap_)){
          dubins_path[0].parent = std::make_shared<nodeHybrid>(current_node);
          cout << "Found path " << i << " max: " << subgoals.size()-1 << endl;
          goal_node.parent = std::make_shared<nodeHybrid>(dubins_path[dubins_path.size()-1]);
          if (subgoal.x == goal_node.x && subgoal.y == goal_node.y) {
            goal_found = true;
          }else{
            open_list.push_back(subgoal);
            last_subgoal = subgoal;
            cout << "It isnt the end goal therefore restarting search from reachable subgoal" << endl;            
          }
          dubins_found = true;
          break;
        }
      }
      if (!dubins_found){
        // start hybrid astar
        cout << "Dubins to goal not found, all is bad" << endl;
        for (int forwards = -1; forwards <= 1; forwards += 2) {
          for (size_t i = 0; i < directions_.size(); i++){
            nodeHybrid successor = nodeHybrid();
            unsigned int mx,my;
            double wx,wy;
            // cout << "-----------------------------------" << endl;
            // cout << "Current node: " << current_node.x << " " << current_node.y << endl;
            costmap_->mapToWorld(current_node.x, current_node.y, wx, wy);
            // cout << "World: " << wx << " " << wy << endl;
            double step_size = 1; // e.g., 10 cm instead of 1 meter
            wx = wx + forwards * step_size * std::cos(current_node.yaw + directions_[i]);
            wy = wy + forwards * step_size * std::sin(current_node.yaw + directions_[i]);
            // cout << "World [i]: " << wx << " " << wy << " " << i << endl;
            costmap_->worldToMap(wx, wy, mx, my);
            
            if (mx >= costmap_->getSizeInCellsX() || my >= costmap_->getSizeInCellsY()) {
              continue;
            }
            if (costmap_->getCost(mx, my) != 0) {
              continue;
            }
            cout << "Map [i]: " << mx << " " << my << " " << i << endl;
            // check if the node is already in the closed list
            successor.x = mx;
            successor.y = my;
            successor.yaw = current_node.yaw + directions_[i];
            successor.g = current_node.g + calculateDist(current_node.x, current_node.y, successor.x, successor.y);
            successor.h = calculateDist(successor.x, successor.y, goal_node.x, goal_node.y) + 10*std::abs(current_node.yaw - successor.yaw);
            successor.f = successor.g + successor.h;
            
            successor.parent = std::make_shared<nodeHybrid>(current_node);
            open_list.push_back(successor);
            
            
          }
        }        
      }
      
    }

    // for (size_t i = 0; i < path_to_consider.size(); i++) {
    //   if(path_to_consider[i].parent == nullptr){
    //     cout << "theres a hole in the path, help needed" << endl;
    //   }
    // }   

    if (path_to_consider.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Dubins path is empty, falling back to straight line");
      path_to_consider.push_back(start_node);
      path_to_consider.push_back(goal_node);
    }
    //path_to_consider = subgoals;
    nodeHybrid path_node = goal_node;
    #if DEBUG == 0
      while (path_node.parent != nullptr) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = node_->now();
        pose.header.frame_id = global_frame_;
        pose.pose.position.x = costmap_->getOriginX() + path_node.x * costmap_->getResolution();
        pose.pose.position.y = costmap_->getOriginY() + path_node.y * costmap_->getResolution();
        pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), path_node.yaw));
        // cout << "Pose: " << pose.pose.position.x << " " << pose.pose.position.y << endl;
        global_path.poses.insert(global_path.poses.begin(), pose);
        path_node = *path_node.parent;
      }
    #else
      for (size_t i = 0; i < path_to_consider.size(); i++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = node_->now();
        pose.header.frame_id = global_frame_;
        pose.pose.position.x = costmap_->getOriginX() + path_to_consider[i].x * costmap_->getResolution();
        pose.pose.position.y = costmap_->getOriginY() + path_to_consider[i].y * costmap_->getResolution();
        pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), path_to_consider[i].yaw));
        cout << "Pose: " << pose.pose.position.x << " " << pose.pose.position.y << endl;

        global_path.poses.insert(global_path.poses.begin(), pose);

      }
    #endif
 
    return global_path;
  }
}  // namespace astar_planner

#include "pluginlib/class_list_macros.hpp"
  PLUGINLIB_EXPORT_CLASS(astar_planner::Astar, nav2_core::GlobalPlanner)