#ifndef __dubins__hpp__
#define __dubins__hpp__

typedef struct
{
    double qi[3];       // the initial configuration
    double param[3];    // the lengths of the three segments
    double rho;         // model forward velocity / model angular velocity
    int type;           // path type. one of LSL, LSR, ...
} DubinsPath;

typedef int (*DubinsWord)(double, double, double, double* );
extern DubinsWord dubins_words[];

#endif // __dubins__hpp__