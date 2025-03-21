
#include "astar_planner/voronoi.hpp"


#include <memory>
#include <vector>
#include <fstream>
#include "astar_planner/node2d.hpp"
#include "astar_planner/nodeHybrid.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"


std::vector<std::vector<nodeHybrid>> read_nodes(string filename, nav2_costmap_2d::Costmap2D * costmap_)
{
    // open file

    // read file and store in node_map
    // file struture is index x y numofneighbours neighbour1x neighbour1y neighbour2x neighbour2y ... neighbourNx neighbourNy
    // first line is the number of nodes
    // node map is a 2d vector of hybrid nodes indexed by x and y positions
    // x and y is in aboslute coordinates, and need to be converted to costmap coordinates
    std::vector<std::vector<nodeHybrid>> node_map;
    return node_map;
    std::ifstream file;
    file.open(filename);
    if (!file.is_open()) {
        return;
    }
    int num_nodes;
    file >> num_nodes;
    
    for (int i = 0; i < num_nodes; i++) {
        int x, y, num_neighbours;
        file >> x >> y >> num_neighbours;
        unsigned int costmap_x, costmap_y;
        std::vector<std::pair<int, int>> neighbours;
        if (!costmap_->worldToMap(x, y, costmap_x, costmap_y)) {
            
        }
        for (int j = 0; j < num_neighbours; j++) {
            int neighbour_x, neighbour_y;
            file >> neighbour_x >> neighbour_y;
            unsigned int costmap_neighbour_x, costmap_neighbour_y;
            if (!costmap_->worldToMap(neighbour_x, neighbour_y, costmap_neighbour_x, costmap_neighbour_y)) {
                
            }
            neighbours.push_back(std::make_pair(costmap_neighbour_x, costmap_neighbour_y));
        }
        nodeHybrid node = nodeHybrid(costmap_x, costmap_y, 0, 0, 0, nullptr, neighbours);
        node_map[costmap_x][costmap_y] = node;

    }
    file.close();
    return node_map;
    

    // close file
    
}
