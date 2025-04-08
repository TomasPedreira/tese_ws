#ifndef __dubins__hpp__
#define __dubins__hpp__


#include "astar_planner/nodeHybrid.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"





int dubins_LSL(double alpha, double beta, double d, double* outputs);
int dubins_LSR(double alpha, double beta, double d, double* outputs);
int dubins_RSL(double alpha, double beta, double d, double* outputs);
int dubins_RSR(double alpha, double beta, double d, double* outputs);
int dubins_RLR(double alpha, double beta, double d, double* outputs);
int dubins_LRL(double alpha, double beta, double d, double* outputs);
double fmodr(double x, double y);
double mod2pi(double theta);

std::vector<nodeHybrid> create_dubins_path(
    nav2_costmap_2d::Costmap2D* costmap,
    const nodeHybrid &start_node, 
    const nodeHybrid &goal_node,
    double rho,
    double tolerance
);



#endif // __dubins__hpp__