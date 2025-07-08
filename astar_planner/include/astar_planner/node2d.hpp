#ifndef __node2d__hpp__
#define __node2d__hpp__

#include <memory>

using namespace std;

class node2d {
    public:
        unsigned int x, y;
        double f, g, h, yaw;
        std::shared_ptr<node2d> parent;

        node2d() {};

        node2d(unsigned int x, unsigned int y, double f, double g, double h, std::shared_ptr<node2d> parent)
        {
            this->x = x;
            this->y = y;
            this->f = f;
            this->g = g;
            this->h = h;
            this->parent = parent;
            this->yaw = 0.0;
        };

        bool operator>(const node2d & other) const
        {
            return f > other.f;
        }
        bool operator<(const node2d & other) const
        {
            return f < other.f;
        }

        bool operator==(const node2d & other) const
        {
            return x == other.x && y == other.y;
        }
        
};

#endif  // __node2d__hpp__