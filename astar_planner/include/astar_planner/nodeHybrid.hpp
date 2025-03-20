#ifndef __nodeHybrid__hpp__
#define __nodeHybrid__hpp__


#include <memory>

using namespace std;

class nodeHybrid {
    public:
        unsigned int x, y;
        double f, g, h, yaw, steer;
        std::shared_ptr<nodeHybrid> parent;
        std::vector<double> steer_list;

        nodeHybrid() {};

        nodeHybrid(unsigned int x, unsigned int y, double f, double g, double h, std::shared_ptr<nodeHybrid> parent)
        {
            this->x = x;
            this->y = y;
            this->f = f;
            this->g = g;
            this->h = h;
            this->parent = parent;
            this->yaw = 0.0;
        };

        bool operator>(const nodeHybrid & other) const
        {
            return f > other.f;
        }
        bool operator<(const nodeHybrid & other) const
        {
            return f < other.f;
        }

        bool operator==(const nodeHybrid & other) const
        {
            return x == other.x && y == other.y && steer == other.steer;
        }
        
};

#endif // __nodeHybrid__hpp__
