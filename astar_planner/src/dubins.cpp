#include "astar_planner/dubins.hpp"
#include <math.h>
#include <string>
#include <memory>
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <cassert>

#define LSL (0)
#define LSR (1)
#define RSL (2)
#define RSR (3)
#define RLR (4)
#define LRL (5)

// The three segment types a path can be made up of
#define L_SEG (0)
#define S_SEG (1)
#define R_SEG (2)

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

void dubins_segment( double t, double qi[3], double qt[3], int type)
{
    assert( type == L_SEG || type == S_SEG || type == R_SEG );

    if( type == L_SEG ) {
        qt[0] = qi[0] + sin(qi[2]+t) - sin(qi[2]);
        qt[1] = qi[1] - cos(qi[2]+t) + cos(qi[2]);
        qt[2] = qi[2] + t;
    }
    else if( type == R_SEG ) {
        qt[0] = qi[0] - sin(qi[2]-t) + sin(qi[2]);
        qt[1] = qi[1] + cos(qi[2]-t) - cos(qi[2]);
        qt[2] = qi[2] - t;
    }
    else if( type == S_SEG ) {
        qt[0] = qi[0] + cos(qi[2]) * t;
        qt[1] = qi[1] + sin(qi[2]) * t;
        qt[2] = qi[2];
    }
}

std::vector<std::pair<int, int>> dubins_path(
  double x1, double y1, double theta1,
  double x2, double y2, double theta2,
  double radius)
{
  // Implement the Dubins path calculation here
  std::vector<std::pair<int, int>> path;
  // For now, just return a straight line
  path.push_back({static_cast<int>(x1), static_cast<int>(y1)});
  path.push_back({static_cast<int>(x2), static_cast<int>(y2)});
  return path;
}