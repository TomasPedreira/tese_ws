#include "astar_planner/auxiliary_functions.hpp"
#include "astar_planner/nodeHybrid.hpp"
#include <cmath>
#include <vector>


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

    