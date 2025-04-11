#include "astar_planner/dubins.hpp"
#include <math.h>
#include <string>
#include <memory>
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <cassert>

#include "astar_planner/nodeHybrid.hpp"
#include <iostream>
#include <ostream>
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include <rclcpp/rclcpp.hpp>


// The three segment types a path can be made up of
#define L_SEG (0)
#define S_SEG (1)
#define R_SEG (2)

#define LSL (0)
#define LSR (1)
#define RSL (2)
#define RSR (3)
#define RLR (4)
#define LRL (5)

typedef struct
{
    double qi[3];       // the initial configuration
    double param[3];    // the lengths of the three segments
    double rho;         // model forward velocity / model angular velocity
    int type;           // path type. one of LSL, LSR, ...
} DubinsPath;

typedef int (*DubinsWord)(double, double, double, double* );
extern DubinsWord dubins_words[];

DubinsWord dubins_words[] = {
    dubins_LSL,
    dubins_LSR,
    dubins_RSL,
    dubins_RSR,
    dubins_RLR,
    dubins_LRL,
};


double fmodr( double x, double y)
{
    return x - y*floor(x/y);
}

double mod2pi( double theta )
{
    return fmodr( theta, 2 * M_PI );
}

int dubins_LSL( double alpha, double beta, double d, double* outputs )
{
    double sa = sin(alpha); 
    double sb = sin(beta); 
    double ca = cos(alpha);
    double cb = cos(beta); 
    double c_ab = cos(alpha - beta);   
    double tmp0 = d+sa-sb;
    double p_squared = 2 + (d*d) -(2*c_ab) + (2*d*(sa - sb));
    if( p_squared < 0 ) {
        return -1;
    }
    double tmp1 = atan2( (cb-ca), tmp0 );
    double t = mod2pi(-alpha + tmp1 );
    double p = sqrt( p_squared );
    double q = mod2pi(beta - tmp1 );
    outputs[0]  = t;                
    outputs[1]  = p;                
    outputs[2]  = q;
    return 0;
}

int dubins_RSR( double alpha, double beta, double d, double* outputs )
{
    double sa = sin(alpha); double sb = sin(beta); 
    double ca = cos(alpha);
    double cb = cos(beta); 
    double c_ab = cos(alpha - beta);   
    double tmp0 = d-sa+sb;
    double p_squared = 2 + (d*d) -(2*c_ab) + (2*d*(sb-sa));
    if( p_squared < 0 ) {
        return -1;
    }
    double tmp1 = atan2( (ca-cb), tmp0 );
    double t = mod2pi( alpha - tmp1 );
    double p = sqrt( p_squared );
    double q = mod2pi( -beta + tmp1 );
    outputs[0]  = t;                
    outputs[1]  = p;                
    outputs[2]  = q;
    return 0;
}

int dubins_LSR( double alpha, double beta, double d, double* outputs )
{
    double sa = sin(alpha); 
    double sb = sin(beta); 
    double ca = cos(alpha);
    double cb = cos(beta); 
    double c_ab = cos(alpha - beta);   
    double p_squared = -2 + (d*d) + (2*c_ab) + (2*d*(sa+sb));
    if( p_squared < 0 ) {
        return -1;
    }
    double p    = sqrt( p_squared );
    double tmp2 = atan2( (-ca-cb), (d+sa+sb) ) - atan2(-2.0, p);
    double t    = mod2pi(-alpha + tmp2);
    double q    = mod2pi( -mod2pi(beta) + tmp2 );
    outputs[0]  = t;                
    outputs[1]  = p;                
    outputs[2]  = q;
    return 0;
}

int dubins_RSL( double alpha, double beta, double d, double* outputs )
{
    double sa = sin(alpha); 
    double sb = sin(beta); 
    double ca = cos(alpha);
    double cb = cos(beta); 
    double c_ab = cos(alpha - beta);   
    double p_squared = (d*d) -2 + (2*c_ab) - (2*d*(sa+sb));
    if( p_squared< 0 ) {
        return -1;
    }
    double p    = sqrt( p_squared );
    double tmp2 = atan2( (ca+cb), (d-sa-sb) ) - atan2(2.0, p);
    double t    = mod2pi(alpha - tmp2);
    double q    = mod2pi(beta - tmp2);
    outputs[0]  = t;                
    outputs[1]  = p;                
    outputs[2]  = q; 
    return 0;
}

int dubins_RLR( double alpha, double beta, double d, double* outputs )
{
    double sa = sin(alpha); 
    double sb = sin(beta); 
    double ca = cos(alpha);
    double cb = cos(beta); 
    double c_ab = cos(alpha - beta);   
    double tmp_rlr = (6. - d*d + 2*c_ab + 2*d*(sa-sb)) / 8.;
    if( fabs(tmp_rlr) > 1) {
        return -1;
    }
    double p = mod2pi( 2*M_PI - acos( tmp_rlr ) );
    double t = mod2pi(alpha - atan2( ca-cb, d-sa+sb ) + mod2pi(p/2.));
    double q = mod2pi(alpha - beta - t + mod2pi(p));
    outputs[0]  = t;                
    outputs[1]  = p;                
    outputs[2]  = q; 
    return 0;
}

int dubins_LRL( double alpha, double beta, double d, double* outputs )
{
    double sa = sin(alpha); 
    double sb = sin(beta); 
    double ca = cos(alpha);
    double cb = cos(beta); 
    double c_ab = cos(alpha - beta);   
    double tmp_lrl = (6. - d*d + 2*c_ab + 2*d*(- sa + sb)) / 8.;
    if( fabs(tmp_lrl) > 1) {
        return -1;
    }
    double p = mod2pi( 2*M_PI - acos( tmp_lrl ) );
    double t = mod2pi(-alpha - atan2( ca-cb, d+sa-sb ) + p/2.);
    double q = mod2pi(mod2pi(beta) - alpha -t + mod2pi(p));
    outputs[0]  = t;                
    outputs[1]  = p;                
    outputs[2]  = q; 
    return 0;
}

int dubins_init_normalised( double alpha, double beta, double d, DubinsPath* path)
{
    double best_cost = INFINITY;
    int    best_word;
    int    i;

    best_word = -1;
    for( i = 0; i < 6; i++ ) {
        double params[3];
        int err = dubins_words[i](alpha, beta, d, params);
        if(err == 0) {
            double cost = params[0] + params[1] + params[2];
            if(cost < best_cost) {
                best_word = i;
                best_cost = cost;
                path->param[0] = params[0];
                path->param[1] = params[1];
                path->param[2] = params[2];
                path->type = i;
            }
        }
    }

    if(best_word == -1) {
        return -1;
    }
    path->type = best_word;
    return 0;
}

int dubins_init( double q0[3], double q1[3], double rho, DubinsPath* path )
{
    int i;
    double dx = q1[0] - q0[0];
    double dy = q1[1] - q0[1];
    double D = sqrt( dx * dx + dy * dy );
    double d = D / rho;
    if( rho <= 0. ) {
        return -1;
    }
    double theta = mod2pi(atan2( dy, dx ));
    double alpha = mod2pi(q0[2] - theta);
    double beta  = mod2pi(q1[2] - theta);
    for( i = 0; i < 3; i ++ ) {
        path->qi[i] = q0[i];
    }
    path->rho = rho;

    return dubins_init_normalised( alpha, beta, d, path );
}

double dubins_path_length( DubinsPath* path )
{
    double length = 0.;
    length += path->param[0];
    length += path->param[1];
    length += path->param[2];
    length = length * path->rho;
    return length;
}

int dubins_path_type( DubinsPath* path ) {
    return path->type;
}


void dubins_segment(double t, double qi[3], double qt[3], int type, double rho) {
    assert(type == L_SEG || type == S_SEG || type == R_SEG);

    if (type == L_SEG) {
        qt[0] = qi[0] + rho * (sin(qi[2] + t) - sin(qi[2]));
        qt[1] = qi[1] - rho * (cos(qi[2] + t) - cos(qi[2]));
        qt[2] = qi[2] + t;
    } else if (type == R_SEG) {
        qt[0] = qi[0] - rho * (sin(qi[2] - t) - sin(qi[2]));
        qt[1] = qi[1] + rho * (cos(qi[2] - t) - cos(qi[2]));
        qt[2] = qi[2] - t;
    } else if (type == S_SEG) {
        qt[0] = qi[0] + rho * cos(qi[2]) * t;
        qt[1] = qi[1] + rho * sin(qi[2]) * t;
        qt[2] = qi[2];
    }
}
/*
    @brief Create a Dubins path between two nodes with tolerance
    @param costmap: The costmap to use for coordinate conversion
    @param start_node: The starting node of the path
    @param goal_node: The goal node of the path
    @param rho: The turning radius (v/ω), can be < 1
    @param tolerance: Max allowable distance to consider goal reached (in world units)
    @return: A vector of nodes representing the Dubins path from start_node to goal_node
 */
std::vector<nodeHybrid> create_dubins_path(
    nav2_costmap_2d::Costmap2D* costmap,
    const nodeHybrid &start_node, 
    const nodeHybrid &goal_node,
    double rho,
    double tolerance = 0.2
) {
    std::vector<nodeHybrid> path_nodes;

    // 1. Convert map coordinates to world coordinates
    double start_x_world, start_y_world;
    double goal_x_world, goal_y_world;
    costmap->mapToWorld(start_node.x, start_node.y, start_x_world, start_y_world);
    costmap->mapToWorld(goal_node.x, goal_node.y, goal_x_world, goal_y_world);

    if (start_node.x >= costmap->getSizeInCellsX() || start_node.y >= costmap->getSizeInCellsY() ||
        goal_node.x >= costmap->getSizeInCellsX() || goal_node.y >= costmap->getSizeInCellsY()) {
        RCLCPP_ERROR(rclcpp::get_logger("AstarPlanner"), "Start or goal outside costmap bounds: Start(%u, %u), Goal(%u, %u)", 
                     start_node.x, start_node.y, goal_node.x, goal_node.y);
        return path_nodes;
    }

    // 2. Check tolerance
    double dx = goal_x_world - start_x_world;
    double dy = goal_y_world - start_y_world;
    double dist_to_goal = sqrt(dx * dx + dy * dy);
    RCLCPP_DEBUG(rclcpp::get_logger("AstarPlanner"), "Dist to goal: %f, Tolerance: %f, Rho: %f", 
                 dist_to_goal, tolerance, rho);

    if (dist_to_goal <= tolerance) {
        RCLCPP_INFO(rclcpp::get_logger("AstarPlanner"), "Within tolerance (%f <= %f), direct path", dist_to_goal, tolerance);
        path_nodes.push_back(start_node);
        nodeHybrid goal = goal_node;
        goal.x = goal_node.x;
        goal.y = goal_node.y;
        goal.yaw = goal_node.yaw;
        path_nodes.push_back(goal);
        return path_nodes;
    }

    // 3. Initialize Dubins path
    double q0[3] = {start_x_world, start_y_world, start_node.yaw};
    double q1[3] = {goal_x_world, goal_y_world, goal_node.yaw};
    if (rho <= 0.0) {
        RCLCPP_ERROR(rclcpp::get_logger("AstarPlanner"), "Invalid rho: %f", rho);
        return path_nodes;
    }
    DubinsPath path;
    double D = dist_to_goal;
    double d = D / rho;
    // RCLCPP_DEBUG(rclcpp::get_logger("AstarPlanner"), "Dubins init: D=%f, d=%f, rho=%f, start_yaw=%f, goal_yaw=%f", 
    //              D, d, rho, start_node.yaw, goal_node.yaw);

    double theta = mod2pi(atan2(dy, dx));
    double alpha = mod2pi(q0[2] - theta);
    double beta = mod2pi(q1[2] - theta);
    double best_cost = INFINITY;
    int best_word = -1;
    for (int i = 0; i < 6; i++) {
        double params[3];
        int err = dubins_words[i](alpha, beta, d, params);
        if (err == 0) {
            double cost = params[0] + params[1] + params[2];
            // RCLCPP_DEBUG(rclcpp::get_logger("AstarPlanner"), "Word %d (%s): Cost=%f, Params=[%f, %f, %f]", 
            //              i, i == LSL ? "LSL" : i == LSR ? "LSR" : i == RSL ? "RSL" : i == RSR ? "RSR" : i == RLR ? "RLR" : "LRL", 
            //              cost, params[0], params[1], params[2]);
            if (cost < best_cost) {
                best_word = i;
                best_cost = cost;
                path.param[0] = params[0];
                path.param[1] = params[1];
                path.param[2] = params[2];
                path.type = i;
            }
        } else {
            RCLCPP_DEBUG(rclcpp::get_logger("AstarPlanner"), "Word %d failed", i);
        }
    }

    if (best_word == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("AstarPlanner"), "All Dubins words failed with rho=%f, d=%f, alpha=%f, beta=%f", 
                     rho, d, alpha, beta);
        path_nodes.push_back(start_node);
        nodeHybrid goal = goal_node;
        goal.x = goal_node.x;
        goal.y = goal_node.y;
        goal.yaw = goal_node.yaw;
        goal.parent = std::make_shared<nodeHybrid>(start_node);
        path_nodes.push_back(goal);
        return path_nodes;
    }
    path.qi[0] = q0[0];
    path.qi[1] = q0[1];
    path.qi[2] = q0[2];
    path.rho = rho;

    // RCLCPP_INFO(rclcpp::get_logger("AstarPlanner"), "Dubins type=%d (%s), Params=[%f, %f, %f]", 
    //             path.type, path.type == LSL ? "LSL" : path.type == LSR ? "LSR" : path.type == RSL ? "RSL" : 
    //             path.type == RSR ? "RSR" : path.type == RLR ? "RLR" : "LRL", 
    //             path.param[0], path.param[1], path.param[2]);

    // 4. Sample the path
    double path_length = dubins_path_length(&path);
    double step_size = rho * 0.1;
    // RCLCPP_DEBUG(rclcpp::get_logger("AstarPlanner"), "Path length=%f, step_size=%f", path_length, step_size);

    double qi[3] = {q0[0], q0[1], q0[2]};
    double qt[3];
    int segment_types[3];
    switch (path.type) {
        case LSL: segment_types[0] = L_SEG; segment_types[1] = S_SEG; segment_types[2] = L_SEG; break;
        case LSR: segment_types[0] = L_SEG; segment_types[1] = S_SEG; segment_types[2] = R_SEG; break;
        case RSL: segment_types[0] = R_SEG; segment_types[1] = S_SEG; segment_types[2] = L_SEG; break;
        case RSR: segment_types[0] = R_SEG; segment_types[1] = S_SEG; segment_types[2] = R_SEG; break;
        case RLR: segment_types[0] = R_SEG; segment_types[1] = L_SEG; segment_types[2] = R_SEG; break;
        case LRL: segment_types[0] = L_SEG; segment_types[1] = R_SEG; segment_types[2] = L_SEG; break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("AstarPlanner"), "Invalid path type: %d", path.type);
            return path_nodes;
    }

    path_nodes.push_back(start_node);
    double total_traveled = 0.0;
    bool goal_reached = false;

    for (int seg = 0; seg < 3 && !goal_reached; seg++) {
        double seg_length = path.param[seg];
        double real_seg_length = seg_length * rho;
        int seg_steps = std::max(5, static_cast<int>(real_seg_length / step_size) + 1);
        // RCLCPP_DEBUG(rclcpp::get_logger("AstarPlanner"), "Seg %d: Type=%d, NormLength=%f, RealLength=%f, Steps=%d", 
        //              seg, segment_types[seg], seg_length, real_seg_length, seg_steps);

        for (int i = 0; i < seg_steps; i++) {
            double t = seg_length * (i / static_cast<double>(seg_steps - 1));
            if (i == seg_steps - 1) t = seg_length;
            dubins_segment(t, qi, qt, segment_types[seg], rho);

            double dist_from_start = sqrt(pow(qt[0] - start_x_world, 2) + pow(qt[1] - start_y_world, 2));
            total_traveled = dist_from_start;
            if (total_traveled > dist_to_goal + rho) {
                // RCLCPP_WARN(rclcpp::get_logger("AstarPlanner"), "Seg %d, Step %d: Overshoot (%f > %f), adjusting", 
                //             seg, i, total_traveled, dist_to_goal);
                break;
            }

            unsigned int mx, my;
            if (!costmap->worldToMap(qt[0], qt[1], mx, my)) {
                // RCLCPP_WARN(rclcpp::get_logger("AstarPlanner"), "Seg %d, Step %d: Point (%f, %f) outside costmap", 
                //             seg, i, qt[0], qt[1]);
                continue;
            }

            if (costmap->getCost(mx, my) >= 254) {
                // RCLCPP_WARN(rclcpp::get_logger("AstarPlanner"), "Seg %d, Step %d: Point (%u, %u) in collision", 
                //             seg, i, mx, my);
                continue;
            }

            nodeHybrid node;
            node.x = mx;
            node.y = my;
            node.yaw = mod2pi(qt[2]);
            node.parent = std::make_shared<nodeHybrid>(path_nodes.back());
            if (!path_nodes.empty() && node.x == path_nodes.back().x && node.y == path_nodes.back().y) {
                continue;
            }
            path_nodes.push_back(node);
            RCLCPP_DEBUG(rclcpp::get_logger("AstarPlanner"), "Seg %d, Step %d: (%u, %u), yaw=%f, world(%f, %f), dist=%f", 
                         seg, i, mx, my, node.yaw, qt[0], qt[1], dist_from_start);

            double dist_to_goal_now = sqrt(pow(qt[0] - goal_x_world, 2) + pow(qt[1] - goal_y_world, 2));
            if (dist_to_goal_now <= tolerance && seg < 2) {
                RCLCPP_INFO(rclcpp::get_logger("AstarPlanner"), "Seg %d, Step %d: Near goal (%f <= %f), finalizing", 
                            seg, i, dist_to_goal_now, tolerance);
                nodeHybrid goal = goal_node;
                goal.x = goal_node.x;
                goal.y = goal_node.y;
                goal.yaw = goal_node.yaw;
                goal.parent = std::make_shared<nodeHybrid>(path_nodes.back());
                path_nodes.push_back(goal);
                goal_reached = true;
                break;  // Exit inner loop, outer loop will exit due to goal_reached
            }

            if (i == seg_steps - 1) {
                qi[0] = qt[0];
                qi[1] = qt[1];
                qi[2] = qt[2];
            }
        }
    }

    // Post-processing
    if (!path_nodes.empty()) {
        double last_dist = sqrt(pow(qt[0] - goal_x_world, 2) + pow(qt[1] - goal_y_world, 2));
        RCLCPP_DEBUG(rclcpp::get_logger("AstarPlanner"), "Last point to goal dist: %f", last_dist);
        if (last_dist > tolerance) {
            RCLCPP_WARN(rclcpp::get_logger("AstarPlanner"), "Final point %f from goal, forcing goal", last_dist);
            nodeHybrid goal = goal_node;
            goal.x = goal_node.x;
            goal.y = goal_node.y;
            goal.yaw = goal_node.yaw;
            goal.parent = std::make_shared<nodeHybrid>(path_nodes.back());
            path_nodes.push_back(goal);
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("AstarPlanner"), "Path empty, forcing goal");
        nodeHybrid goal = goal_node;
        goal.x = goal_node.x;
        goal.y = goal_node.y;
        goal.yaw = goal_node.yaw;
        goal.parent = std::make_shared<nodeHybrid>(start_node);
        path_nodes.push_back(goal);
    }

    RCLCPP_INFO(rclcpp::get_logger("AstarPlanner"), "Path size: %zu (rho=%f)", path_nodes.size(), rho);
    if (path_nodes.size() < 5) {
        RCLCPP_WARN(rclcpp::get_logger("AstarPlanner"), "Path too short (%zu points), curves may be incomplete", 
                    path_nodes.size());
    }

    return path_nodes;
}

bool dubins_check_colision(std::vector<nodeHybrid> &path_nodes, nav2_costmap_2d::Costmap2D* costmap) {
    for (const auto& node : path_nodes) {
        if (costmap->getCost(node.x, node.y) > 0) {
            return true;
        }
    }
    return false;
}