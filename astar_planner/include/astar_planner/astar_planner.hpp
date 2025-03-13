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

  double max_steer_;
  double tolerance_;
};

}  // namespace astar_planner

#endif  // ASTAR_PLANNER__ASTAR_PLANNER_HPP_
