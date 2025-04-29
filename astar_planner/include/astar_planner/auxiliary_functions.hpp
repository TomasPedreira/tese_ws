#ifndef ASTAR_PLANNER_AUXILIARY_FUNCTIONS_HPP
#define ASTAR_PLANNER_AUXILIARY_FUNCTIONS_HPP

#include "astar_planner/nodeHybrid.hpp"
#include <vector>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>


double calculateDist(double x1, double y1, double x2, double y2);
double calculateOrientation(double x1, double y1, double x2, double y2);
int get_lowest_f_node(std::vector<nodeHybrid> & open_list);
nav_msgs::msg::Path path_from_vector(std::vector<nodeHybrid> & path_nodes, const std::string & global_frame, nav2_costmap_2d::Costmap2D* costmap);
bool is_node_in_list(const nodeHybrid &node, const std::vector<nodeHybrid> &list);
int get_node_from_list(const nodeHybrid &node, const std::vector<nodeHybrid> &list);
bool getRobotTransforms(
    tf2_ros::Buffer & tf_buffer,
    geometry_msgs::msg::TransformStamped & base_link_transform,
    geometry_msgs::msg::TransformStamped & trailer_link_transform,
    const std::string & global_frame
);

#endif // ASTAR_PLANNER_AUXILIARY_FUNCTIONS_HPP