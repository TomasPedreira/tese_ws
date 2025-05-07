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
#include "astar_planner/auxiliary_functions.hpp"


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
    const double speed = 0.4 / costmap->getResolution();
    const double rtr = 0.5625 / costmap->getResolution();

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

    double dx = goal_x_world - start_x_world;
    double dy = goal_y_world - start_y_world;
    double dist_to_goal = sqrt(dx * dx + dy * dy);
    double yaw_diff = mod2pi(goal_node.yaw - start_node.yaw);

    // Check if we're within tolerance
    if (dist_to_goal <= tolerance) {
        path_nodes.push_back(start_node);
        nodeHybrid goal;
        goal.x = goal_node.x;
        goal.y = goal_node.y;
        goal.yaw = goal_node.yaw;
        goal.tx = start_node.x;
        goal.ty = start_node.y;
        goal.trailer_yaw = start_node.trailer_yaw;
        goal.parent = std::make_shared<nodeHybrid>(start_node);
        path_nodes.push_back(goal);
        return path_nodes;
    }

    // Initialize Dubins path
    double q0[3] = {start_x_world, start_y_world, start_node.yaw};
    double q1[3] = {goal_x_world, goal_y_world, goal_node.yaw};
    
    if (rho <= 0.0) {
        RCLCPP_ERROR(rclcpp::get_logger("AstarPlanner"), "Invalid rho: %f", rho);
        return path_nodes;
    }

    DubinsPath path;
    double D = dist_to_goal;
    double d = D / rho;
    double theta = mod2pi(atan2(dy, dx));
    double alpha = mod2pi(q0[2] - theta);
    double beta = mod2pi(q1[2] - theta);

    // Validate direction
    double heading_to_goal = atan2(dy, dx);
    double start_heading_diff = mod2pi(heading_to_goal - start_node.yaw);
    bool should_reverse = std::abs(start_heading_diff) > M_PI/2;

    // Check for obstacles in the direct path
    bool has_obstacle = false;
    unsigned int check_x, check_y;
    double check_step = 0.1; // 10cm steps
    double check_dist = 0.0;
    while (check_dist < dist_to_goal) {
        double ratio = check_dist / dist_to_goal;
        double check_x_world = start_x_world + ratio * dx;
        double check_y_world = start_y_world + ratio * dy;
        if (costmap->worldToMap(check_x_world, check_y_world, check_x, check_y)) {
            if (costmap->getCost(check_x, check_y) >= 254) {
                has_obstacle = true;
                break;
            }
        }
        check_dist += check_step;
    }

    // Try different Dubins words and find the best valid path
    double best_cost = INFINITY;
    int best_word = -1;
    double best_params[3];

    for (int i = 0; i < 6; i++) {
        double params[3];
        int err = dubins_words[i](alpha, beta, d, params);
        if (err == 0) {
            // Basic validation of path parameters
            if (params[0] < 0 || params[1] < 0 || params[2] < 0) {
                continue;
            }

            // Check if the path respects the direction constraint
            bool valid_direction = true;
            if (!should_reverse) {
                if (i == RSL || i == RSR) {
                    valid_direction = false;
                }
            }

            // Check for invalid arc patterns
            bool invalid_pattern = false;
            
            // Only check for invalid patterns when there's an obstacle
            if (has_obstacle) {
                // Check for degenerate cases where arcs would overlap
                if (params[0] < 0.1 && params[2] < 0.1) {
                    invalid_pattern = true;
                }
            }

            if (valid_direction && !invalid_pattern) {
                double cost = params[0] + params[1] + params[2];
                
                // Simple cost adjustments
                if (has_obstacle) {
                    // Slightly prefer paths that start with a turn when obstacles are present
                    if (i == LSL || i == RSR) {
                        cost *= 1.2;
                    }
                }

                if (cost < best_cost) {
                    best_word = i;
                    best_cost = cost;
                    best_params[0] = params[0];
                    best_params[1] = params[1];
                    best_params[2] = params[2];
                }
            }
        }
    }

    if (best_word == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("AstarPlanner"), "No valid Dubins path found");
        return path_nodes;
    }

    // Set up the path
    path.param[0] = best_params[0];
    path.param[1] = best_params[1];
    path.param[2] = best_params[2];
    path.type = best_word;
    path.qi[0] = q0[0];
    path.qi[1] = q0[1];
    path.qi[2] = q0[2];
    path.rho = rho;

    // Sample the path
    double step_size = rho * 0.1;
    double qi[3] = {q0[0], q0[1], q0[2]};
    double qt[3];
    int segment_types[3];
    
    // Set segment types based on path type
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

    // Generate path points for each segment
    for (int seg = 0; seg < 3 && !goal_reached; seg++) {
        double seg_length = path.param[seg];
        double real_seg_length = seg_length * rho;
        int seg_steps = std::max(5, static_cast<int>(real_seg_length / step_size) + 1);

        for (int i = 0; i < seg_steps; i++) {
            double t = seg_length * (i / static_cast<double>(seg_steps - 1));
            if (i == seg_steps - 1) t = seg_length;
            
            dubins_segment(t, qi, qt, segment_types[seg], rho);

            // Check if we've overshot
            double dist_from_start = sqrt(pow(qt[0] - start_x_world, 2) + pow(qt[1] - start_y_world, 2));
            total_traveled = dist_from_start;
            if (total_traveled > dist_to_goal + rho) {
                break;
            }

            // Convert to map coordinates
            unsigned int mx, my;
            if (!costmap->worldToMap(qt[0], qt[1], mx, my)) {
                continue;
            }

            if (costmap->getCost(mx, my) >= 254) {
                continue;
            }

            // Create new node
            nodeHybrid node;
            node.x = mx;
            node.y = my;
            node.yaw = mod2pi(qt[2]);
            node.parent = std::make_shared<nodeHybrid>(path_nodes.back());
            calc_trailer_config(node, *(node.parent), speed, rtr, costmap->getResolution(), 1);

            // Skip if we're at the same position
            if (!path_nodes.empty() && node.x == path_nodes.back().x && node.y == path_nodes.back().y) {
                continue;
            }

            path_nodes.push_back(node);

            // Check if we're close enough to goal
            double dist_to_goal_now = sqrt(pow(qt[0] - goal_x_world, 2) + pow(qt[1] - goal_y_world, 2));
            if (dist_to_goal_now <= tolerance && seg < 2) {
                nodeHybrid goal;
                goal.x = goal_node.x;
                goal.y = goal_node.y;
                goal.yaw = goal_node.yaw;
                calc_trailer_config(goal, node, speed, rtr, costmap->getResolution(), 1);
                goal.parent = std::make_shared<nodeHybrid>(path_nodes.back());
                path_nodes.push_back(goal);
                goal_reached = true;
                break;
            }

            if (i == seg_steps - 1) {
                qi[0] = qt[0];
                qi[1] = qt[1];
                qi[2] = qt[2];
            }
        }
    }

    // Add goal if not reached
    if (!goal_reached && !path_nodes.empty()) {
        nodeHybrid goal;
        goal.x = goal_node.x;
        goal.y = goal_node.y;
        goal.yaw = goal_node.yaw;
        goal.parent = std::make_shared<nodeHybrid>(path_nodes.back());
        calc_trailer_config(goal, *(goal.parent), speed, rtr, costmap->getResolution(), 1);
        path_nodes.push_back(goal);
    }

    return path_nodes;
}

// bool dubins_check_colision(std::vector<nodeHybrid> &path_nodes, nav2_costmap_2d::Costmap2D* costmap) {
//     for (const auto& node : path_nodes) {
//         if (costmap->getCost(node.x, node.y) > 0) {
//             return true;
//         }
//     }
//     return false;
// }
bool dubins_check_colision(std::vector<nodeHybrid> &path_nodes, nav2_costmap_2d::Costmap2D* costmap) {
    
    for (size_t i = 1; i < path_nodes.size(); i++) {
        if (costmap->getCost(path_nodes[i].x, path_nodes[i].y) > 0) {
            // std::cout << "Tractor Collision at node " << i << ": (" << path_nodes[i].x << ", " << path_nodes[i].y << ")" << std::endl;
            return true;
        }
        
        // wrap to 0,2pi
        double yaw_diff = -path_nodes[i].yaw + path_nodes[i].trailer_yaw;
        yaw_diff = abs(atan2(sin(yaw_diff), cos(yaw_diff)));
        if (yaw_diff > M_PI/4) {
            // std::cout << "Yaw diff > pi at node " << i << ": " << yaw_diff << std::endl;
            return true;
        }
        if (costmap->getCost(path_nodes[i].tx, path_nodes[i].ty) > 0) {
            // std::cout << "Collision at node " << i << ": (" << path_nodes[i].tx << ", " << path_nodes[i].ty << ")" << std::endl;
            return true;
        }
        // std::cout << "trailer x: " << path_nodes[i].tx << ", trailer y: " << path_nodes[i].ty << std::endl;
        

    }
    return false;
}