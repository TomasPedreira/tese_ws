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

    // Final angle tolerance parameter
    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".final_angle_tolerance", rclcpp::ParameterValue(0.1));
    node_->get_parameter(name_ + ".final_angle_tolerance", final_angle_tolerance_);

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

    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".step_size", rclcpp::ParameterValue(0.5));
    node_->get_parameter(name_ + ".step_size", step_size_);

    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".timeout", rclcpp::ParameterValue(7.5));
    node_->get_parameter(name_ + ".timeout", timeout_);
    

    // Number of hybrid segments parameter
    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".num_hybrid_segments", rclcpp::ParameterValue(20));
    node_->get_parameter(name_ + ".num_hybrid_segments", num_hybrid_segments_);

    double step = max_angle_ / num_directions_;
    
    for (int i = -num_directions_ + 1; i < num_directions_; i++) {
      double angle = step * i;
      directions_.push_back(angle);
      
    }
    node_expansion_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
      "/node_expansion", 10);
    trailer_pos_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
      "/trailer_pos", 10);
    voronoi_subgoals_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
      "/voronoi_subgoals", 10);
    hybrid_path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
      "/hybrid_nodes", 10);
    dubins_path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
      "/dubins_nodes", 10);
      
  }

  void Astar::cleanup()
  {
    RCLCPP_INFO(
      node_->get_logger(), "CleaningUp plugin %s of type AstarPlanner",
      name_.c_str());
    node_expansion_publisher_.reset();
    trailer_pos_publisher_.reset();
    voronoi_subgoals_publisher_.reset();
    hybrid_path_publisher_.reset();
    dubins_path_publisher_.reset();
  }

  void Astar::activate()
  {
    RCLCPP_INFO(
      node_->get_logger(), "Activating plugin %s of type AstarPlanner",
      name_.c_str());
    node_expansion_publisher_->on_activate();
    trailer_pos_publisher_->on_activate();
    voronoi_subgoals_publisher_->on_activate();
    hybrid_path_publisher_->on_activate();
    dubins_path_publisher_->on_activate();
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
    node_expansion_publisher_->on_deactivate();
    trailer_pos_publisher_->on_deactivate();
    voronoi_subgoals_publisher_->on_deactivate();
    hybrid_path_publisher_->on_deactivate();
    dubins_path_publisher_->on_deactivate();
  }


  nav_msgs::msg::Path Astar::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal
  )
  {
    nav_msgs::msg::Path global_path;

    geometry_msgs::msg::TransformStamped base_link_transform;
    geometry_msgs::msg::TransformStamped trailer_link_transform;
    bool correc_transform = getRobotTransforms(
      *tf_, base_link_transform, trailer_link_transform,  global_frame_);
    if (!correc_transform) {
      RCLCPP_ERROR(
        node_->get_logger(), "Could not get the robot transforms");
      return global_path;
    }
    else{
      cout << "Base link transform: " << base_link_transform.transform.translation.x << " " << base_link_transform.transform.translation.y << endl;
      cout << "Trailer link transform: " << trailer_link_transform.transform.translation.x << " " << trailer_link_transform.transform.translation.y << endl;
      cout << "Base link yaw: " << tf2::getYaw(base_link_transform.transform.rotation) << endl;
      cout << "Trailer link yaw: " << tf2::getYaw(trailer_link_transform.transform.rotation) << endl;
    }
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
    unsigned int trailer_x, trailer_y;
    costmap_->worldToMap(
        trailer_link_transform.transform.translation.x, 
        trailer_link_transform.transform.translation.y, 
        trailer_x, trailer_y);
    nodeHybrid start_node = nodeHybrid();
    start_node.x = start_x;
    start_node.y = start_y;
    start_node.tx = trailer_x;
    start_node.ty = trailer_y;
    start_node.trailer_yaw = tf2::getYaw(trailer_link_transform.transform.rotation);
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
    const double rtr = 0.5625 / costmap_->getResolution();
    const double speed = 0.4 / costmap_->getResolution();
   
    std::vector<nodeHybrid> subgoals = compute_subgoals(voronoi_nodes_, start_node, goal_node, costmap_, tolerance_);
    
    // Publish Voronoi subgoals
    if (voronoi_subgoals_publisher_ && voronoi_subgoals_publisher_->is_activated()) {
      voronoi_subgoals_publisher_->publish(path_from_vector(subgoals, global_frame_, costmap_));
    }
    
    std::vector<nodeHybrid> path_to_consider;
    nodeHybrid current_node = start_node;
    nodeHybrid last_subgoal = start_node;
    std::vector<nodeHybrid> open_list;
    std::vector<nodeHybrid> closed_list;
    std::vector<nodeHybrid> expanded_nodes;
    std::vector<nodeHybrid> hybrid_nodes;
    std::vector<nodeHybrid> dubins_nodes;
    open_list.push_back(current_node);

    bool dubins_found = false;
    bool goal_found = false;
    std::vector<nodeHybrid> trailer_path;
    std::vector<nodeHybrid> dubins_path;
    // start timer
    auto start_time = node_->now();
    while (!open_list.empty() && !goal_found) {
      // check timer for 10 seconds
      auto current_time = node_->now();
      cout << "current time in milliseconds: " << current_time.nanoseconds()/1000 - start_time.nanoseconds()/1000 << endl;
      if (current_time - start_time > rclcpp::Duration::from_seconds(timeout_)) {
        cout << "Timeout, returning last known path" << endl;
        if (last_path_.empty()){
          cout << "No last path found, returning empty path" << endl;
          return global_path;
        }
        global_path = path_from_vector(closed_list, global_frame_, costmap_);
        cout << "Returning path of size: " << global_path.poses.size() << endl;
        return global_path;
      }
      int index = get_lowest_f_node(open_list);
      if (index == -2){
        cout << "Open list is crazy!" << endl;
        break;
      }
      if (index == -1){
        cout << "Open list is empty!" << endl;
        break;
      }
      current_node = open_list[index];
      open_list.erase(open_list.begin() + index);
      closed_list.push_back(current_node);
      dubins_found = false;
      // cout << "started subnode loop" << endl;
      for (int i = subgoals.size()-1; i > -1; i--) {
        nodeHybrid subgoal = subgoals[i];
        if (subgoal.x == last_subgoal.x && subgoal.y == last_subgoal.y) {
          break;
        }
        if (is_node_in_list(subgoal, closed_list)){
          continue;
        }
        // cout << "creating dubins path" << endl;
        dubins_path = create_dubins_path(costmap_, current_node, subgoal, turning_radius_, dubins_tolerance_);
        // cout << "dubins path created" << endl;
        if (!dubins_check_colision(dubins_path, costmap_)) {
          // cout << "Dubins path found!" << endl;
          dubins_nodes.insert(dubins_nodes.end(), dubins_path.begin(), dubins_path.end());
          
          dubins_path[0].parent = std::make_shared<nodeHybrid>(current_node);
          if (subgoal.x == goal_node.x && subgoal.y == goal_node.y) {
            goal_node.parent = std::make_shared<nodeHybrid>(dubins_path[dubins_path.size()-1]);
            goal_found = true;
            // cout << "Goal found through dubins!" << endl;
            calc_trailer_config(goal_node, *goal_node.parent, speed, rtr, costmap_->getResolution(), 1);
            nodeHybrid path_node = goal_node;
          }else{
            subgoal.parent = std::make_shared<nodeHybrid>(dubins_path[dubins_path.size()-1]);            
            double trailer_offset = 0.4625 / costmap_->getResolution();
            subgoal.tx = subgoal.x - trailer_offset * cos(subgoal.parent->yaw);
            subgoal.ty = subgoal.y - trailer_offset * sin(subgoal.parent->yaw);
            subgoal.trailer_yaw = subgoal.parent->trailer_yaw;
            open_list.push_back(subgoal);
            last_subgoal = subgoal;
          }
          dubins_found = true;
          // cout << "ended dubins loop" << endl;
          break;
        }
      }
      // cout << "ended subnode loop" << endl;
      if (!dubins_found){
        // cout << "starting hybrid astar loop" << endl;
        for (int forwards = -1; forwards <= 1; forwards += 2) {
          for (size_t i = 0; i < directions_.size(); i++){
            std::vector<nodeHybrid> hybrid_path = create_hybrid_segment(current_node, goal_node, costmap_, directions_[i], forwards, step_size_, num_hybrid_segments_);
            // cout << "path length: " << hybrid_path.size() << endl;
            if (!dubins_check_colision(hybrid_path, costmap_)){
              nodeHybrid last_node = hybrid_path.back();
              // cout << "last node: " << last_node.x << " " << last_node.y << endl;
              if (!is_node_in_list(last_node, closed_list) && !is_node_in_list(last_node, open_list)) {
                hybrid_path[0].parent = std::make_shared<nodeHybrid>(current_node);
                expanded_nodes.insert(expanded_nodes.end(), hybrid_path.begin(), hybrid_path.end());
                // Publish expanded nodes
                if (node_expansion_publisher_ && node_expansion_publisher_->is_activated()) {
                  node_expansion_publisher_->publish(path_from_vector(expanded_nodes, global_frame_, costmap_));
                }
                current_node = expanded_nodes.back();
                // cout << "pushed node to open list: " << expanded_nodes.back().x << " " << expanded_nodes.back().y << endl;
                open_list.push_back(expanded_nodes.back());
              }
              if (is_node_in_list(last_node, closed_list)){
                int index = get_node_from_list(last_node, closed_list);
                if(index == -1){
                  // cout << "Node not found in closed list!" << endl;
                  continue;
                }
                if(last_node.f < closed_list[index].f){
                  closed_list[index].parent = std::make_shared<nodeHybrid>(current_node);
                  closed_list[index].f = last_node.f;
                  closed_list[index].g = last_node.g;
                  closed_list[index].h = last_node.h;
                  closed_list[index].yaw = last_node.yaw;
                  closed_list[index].tx = last_node.tx;
                  closed_list[index].ty = last_node.ty;
                  closed_list[index].trailer_yaw = last_node.trailer_yaw;
                  closed_list[index].is_hybrid = true;
                }
              }
            }
          }  
        }
        // cout << "ended loop" << endl;
      }     
    }
    // cout << "goal found: " << goal_found << endl;

    if (!goal_found){
      cout << "No path found, returning last path" << endl;
      if (last_path_.empty()){
        cout << "No last path found, returning empty path" << endl;
        return global_path;
      }
      global_path = path_from_vector(closed_list, global_frame_, costmap_);
      cout << "Returning path of size: " << global_path.poses.size() << endl;
      return global_path;
    }
    // Collect nodes from the final path
    std::vector<nodeHybrid> final_hybrid_nodes;
    std::vector<nodeHybrid> final_dubins_nodes;
    nodeHybrid path_node = goal_node;
    cout << "collecting nodes from final path" << endl;
    // add node to the last path
    while (path_node.parent != nullptr) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
      pose.pose.position.x = costmap_->getOriginX() + path_node.x * costmap_->getResolution();
      pose.pose.position.y = costmap_->getOriginY() + path_node.y * costmap_->getResolution();
      pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), path_node.yaw));
      global_path.poses.insert(global_path.poses.begin(), pose);
      
      // Add node to appropriate final path vector based on its type
      if (path_node.is_hybrid) {
        final_hybrid_nodes.push_back(path_node);
      } else if (path_node.is_dubins) {
        final_dubins_nodes.push_back(path_node);
      }

      last_path_.push_back(path_node);
      
      nodeHybrid trailer_node = nodeHybrid();
      trailer_node.x = path_node.tx;
      trailer_node.y = path_node.ty;
      trailer_node.yaw = path_node.trailer_yaw;
      trailer_path.push_back(trailer_node);
      path_node = *path_node.parent;
    }

    
    
    // Publish only the final paths
    if (hybrid_path_publisher_ && hybrid_path_publisher_->is_activated()) {
      hybrid_path_publisher_->publish(path_from_vector(final_hybrid_nodes, global_frame_, costmap_));
    }
    if (dubins_path_publisher_ && dubins_path_publisher_->is_activated()) {
      dubins_path_publisher_->publish(path_from_vector(final_dubins_nodes, global_frame_, costmap_));
    }
    trailer_pos_publisher_->publish(path_from_vector(trailer_path, global_frame_, costmap_));
    cout << "returning global path" << endl;
    return global_path;
  }
}  // namespace astar_planner

#include "pluginlib/class_list_macros.hpp"
  PLUGINLIB_EXPORT_CLASS(astar_planner::Astar, nav2_core::GlobalPlanner)