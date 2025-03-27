#ifndef __voronoi_hpp__
#define __voronoi_hpp__

namespace voronoi
{
    std::vector<std::vector<nodeHybrid>> read_nodes(string filename, std::vector<std::vector<nodeHybrid>> & node_map, nav2_costmap_2d::Costmap2D * costmap_);
}

#endif // __voronoi_hpp__