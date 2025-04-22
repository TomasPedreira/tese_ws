#ifndef ASTAR_PLANNER_AUXILIARY_FUNCTIONS_HPP
#define ASTAR_PLANNER_AUXILIARY_FUNCTIONS_HPP

#include "astar_planner/nodeHybrid.hpp"
#include <vector>

double calculateDist(double x1, double y1, double x2, double y2);
double calculateOrientation(double x1, double y1, double x2, double y2);
int get_lowest_f_node(std::vector<nodeHybrid> & open_list);

#endif // ASTAR_PLANNER_AUXILIARY_FUNCTIONS_HPP