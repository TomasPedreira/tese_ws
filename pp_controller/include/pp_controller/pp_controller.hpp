#ifndef PP_CONTROLLER_HPP_
#define PP_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace pp_controller
{

class PPController : public nav2_core::Controller
{
public:
    PPController() = default;
    ~PPController() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;

    void setPlan(const nav_msgs::msg::Path & path) override;
    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav_msgs::msg::Path global_plan_;
    nav_msgs::msg::Path trailer_plan_;
    double lookahead_distance_;
    double max_linear_speed_;
    double max_angular_speed_;
    double collision_check_distance_;  
    bool use_collision_check_;
    bool is_initialized_;
    int last_waypoint_index_;

    // Helper method to check for collisions
    bool checkForCollisions(const geometry_msgs::msg::PoseStamped & current_pose, 
                          const geometry_msgs::msg::PoseStamped & goal_pose);

    // trailer position publisher
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr trailer_position_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr control_goal_pose_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_trailer_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr collision_check_publisher_;

    // Helper methods
    double calculate_angvel_reverse(geometry_msgs::msg::PoseStamped goal_trailer_pose, geometry_msgs::msg::PoseStamped current_trailer_pose, geometry_msgs::msg::PoseStamped current_tractor_pose,const double max_linear_speed, double lookahead_distance);
    double calculate_angvel_forward(geometry_msgs::msg::PoseStamped goal_pose, geometry_msgs::msg::PoseStamped current_pose, double lookahead_distance);
    geometry_msgs::msg::PoseStamped calc_trailer_config(  geometry_msgs::msg::PoseStamped &tractor_pose, geometry_msgs::msg::PoseStamped &previous_trailer_pose, geometry_msgs::msg::PoseStamped &previous_tractor_pose, int dir);
};

} // namespace pp_controller

#endif // PP_CONTROLLER_HPP_ 