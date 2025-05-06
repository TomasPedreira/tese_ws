#include "astar_planner/auxiliary_functions.hpp"
#include "astar_planner/nodeHybrid.hpp"
#include <cmath>
#include <vector>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <iostream>


double calculateDist(double x1, double y1, double x2, double y2)
{
    return std::hypot(x2 - x1, y2 - y1);
}
double calculateOrientation(double x1, double y1, double x2, double y2)
{
    return std::atan2(y2 - y1, x2 - x1);
}

int get_lowest_f_node(std::vector<nodeHybrid> & open_list)
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

nav_msgs::msg::Path path_from_vector(std::vector<nodeHybrid> & path_nodes, const std::string & global_frame, nav2_costmap_2d::Costmap2D* costmap)
{
    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Clock().now();
    path.header.frame_id = global_frame;
    for (const auto & node : path_nodes) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = path.header.stamp;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = costmap->getOriginX() + node.x * costmap->getResolution();
        pose.pose.position.y = costmap->getOriginY() + node.y * costmap->getResolution();
        
        pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), node.yaw));
        path.poses.push_back(pose);
    }
    return path;
}

bool is_node_in_list(const nodeHybrid &node, const std::vector<nodeHybrid> &list)
{
    for (const auto &n : list) {
        if (n.x == node.x && n.y == node.y) {
            return true;
        }
    }
    return false;
}
int get_node_from_list(const nodeHybrid &node, const std::vector<nodeHybrid> &list)
{
    for (size_t i = 0; i < list.size(); i++) {
        if (list[i].x == node.x && list[i].y == node.y) {
            return i;
        }
    }
    return -1;
}

double calculate_trailer_yaw_rate (double speed,double rtr, double tractor_yaw, double trailer_yaw)
{
    double trailer_yaw_rate = speed * sin(trailer_yaw - tractor_yaw) / rtr;
    return trailer_yaw_rate;
}
double calculate_tractor_yaw_rate (double speed, double wb, double steering)
{
    double tractor_yaw_rate = speed * tan(steering) / wb;
    return tractor_yaw_rate;
}
bool getRobotTransforms(
    tf2_ros::Buffer & tf_buffer,
    geometry_msgs::msg::TransformStamped & base_link_transform,
    geometry_msgs::msg::TransformStamped & trailer_link_transform,
    const std::string & global_frame = "map"
){
    try {
        // Get transform from global frame to base_link
        base_link_transform = tf_buffer.lookupTransform(
        global_frame, "base_link", tf2::TimePointZero,
        tf2::durationFromSec(0.5));

        // Get transform from global frame to trailer_link
        trailer_link_transform = tf_buffer.lookupTransform(
        global_frame, "trailer_link", tf2::TimePointZero,
        tf2::durationFromSec(0.5));

        return true;
    }
    catch (tf2::TransformException & ex) {
        std::cout << "Transform error: " << ex.what() << std::endl;
        return false;
    }
}

void calc_trailer_config(nodeHybrid &node, nodeHybrid parent, double speed, double rtr, double resolution){
    double distance = calculateDist(node.x, node.y, parent.x, parent.y);
    double time = distance / speed;
    double trailer_offset = 0.4625 / resolution;
    double trailer_x_disp = 0.15 / resolution;
    node.tx = node.x - (trailer_offset + trailer_x_disp) * cos(node.yaw);
    node.ty = node.y - trailer_offset * sin(node.yaw);
    double new_trailer_yaw = 0.0;
    if (is_node_behind(node, parent)) {
        new_trailer_yaw = parent.trailer_yaw + (-speed/rtr)*sin(-parent.trailer_yaw + parent.yaw) * time;
    }else{
        new_trailer_yaw = parent.trailer_yaw + (speed/rtr)*sin(-parent.trailer_yaw + parent.yaw) * time;
    }
    node.trailer_yaw = new_trailer_yaw;
}

bool is_node_behind(const nodeHybrid &node, const nodeHybrid &previous)
{
    double dx = node.x - previous.x;
    double dy = node.y - previous.y;
    if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) {
        return false;
    }
    
    double headingx = cos(previous.yaw);
    double headingy = sin(previous.yaw);
    double dot_product = dx * headingx + dy * headingy;
    

    return dot_product > 0;
}