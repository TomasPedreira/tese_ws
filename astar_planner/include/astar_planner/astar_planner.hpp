#ifndef ASTAR_PLANNER__ASTAR_PLANNER_HPP_
#define ASTAR_PLANNER__ASTAR_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <queue>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "astar_planner/nodeHybrid.hpp"



namespace astar_planner
{

class Astar : public nav2_core::GlobalPlanner
{
public:
  Astar() = default;
  ~Astar() = default;

  // Overridden methods from the base class
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // A* algorithm specific methods
  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // node ptr
  nav2_util::LifecycleNode::SharedPtr node_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;

  std::vector<nodeHybrid> voronoi_nodes_;

  double tolerance_;

  double timeout_;

  std::vector<nodeHybrid> last_path_;

  // Dubins variables
  double turning_radius_;
  double dubins_tolerance_;
  double final_angle_tolerance_;  // Tolerance for the final angle in radians

  // Hybrid astar variables
  double max_angle_;
  double step_size_;
  std::vector<double> directions_;
  int num_directions_;
  bool allow_reverse_;

  int num_hybrid_segments_;

  // File paths
  std::string nodes_file_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr node_expansion_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr trailer_pos_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr voronoi_subgoals_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr dubins_path_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr hybrid_path_publisher_;


};

}  // namespace astar_planner

#endif  // ASTAR_PLANNER__ASTAR_PLANNER_HPP_
