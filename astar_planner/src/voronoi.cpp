
#include "astar_planner/voronoi.hpp"


#include <memory>
#include <vector>
#include <fstream>
#include "astar_planner/node2d.hpp"
#include "astar_planner/nodeHybrid.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace voronoi
{

// std::vector<std::vector<nodeHybrid>> read_nodes(
//     string filename, 
//     std::vector<std::vector<nodeHybrid>> & node_map,
//     nav2_costmap_2d::Costmap2D * costmap_
// )
//   {
//       std::ifstream file;
//       file.open(filename);
//       if (!file.is_open()) {
//           return node_map;
//       }
//       int num_nodes;
//       file >> num_nodes;

//       for (int i = 0; i < num_nodes; i++) {
//           float x, y;
//           int index, num_neighbours;
//           file >> index >> x >> y >> num_neighbours;
//           unsigned int costmap_x, costmap_y;
//           std::vector<std::pair<int, int>> neighbours;

//           if (!costmap_->worldToMap(x, y, costmap_x, costmap_y)) {
              
//           }
//           for (int j = 0; j < num_neighbours; j++) {
//               float neighbour_x, neighbour_y;
//               file >> neighbour_x >> neighbour_y;
//               unsigned int costmap_neighbour_x, costmap_neighbour_y;
//               if (!costmap_->worldToMap(neighbour_x, neighbour_y, costmap_neighbour_x, costmap_neighbour_y)) {
                  
//               }
//               neighbours.push_back(std::make_pair(costmap_neighbour_x, costmap_neighbour_y));
//           }

//           nodeHybrid node = nodeHybrid(costmap_x, costmap_y, 0, 0, 0, nullptr, neighbours);
//           node_map[costmap_x][costmap_y] = node;
//       }
//       file.close();
//       return node_map;

//   }
}