#ifndef __voronoi_hpp__
#define __voronoi_hpp__


std::vector<nodeHybrid> read_nodes(const std::string& filename, nav2_costmap_2d::Costmap2D* costmap_);
std::vector<int> get_neighbours(const std::vector<nodeHybrid>& voronoi_nodes, const nodeHybrid& start_node, nav2_costmap_2d::Costmap2D* costmap_, int k) ;
std::vector<nodeHybrid> compute_subgoals (
    const std::vector<nodeHybrid> & voronoi_nodes,
    const nodeHybrid & start_node,
    const nodeHybrid & goal_node,
    nav2_costmap_2d::Costmap2D* costmap_,
    double tolerance
  );

#endif // __voronoi_hpp__