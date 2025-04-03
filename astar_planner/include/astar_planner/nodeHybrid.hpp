#ifndef __nodeHybrid__hpp__
#define __nodeHybrid__hpp__


#include <memory>
#include <vector>

class nodeHybrid {
    public:
        unsigned int x, y;
        int id;
        double f, g, h, yaw;
        std::shared_ptr<nodeHybrid> parent;
        std::vector<int> neighbours;
        bool is_outside;
        int num_interpolations;

        nodeHybrid() {};

        nodeHybrid(int id, bool outside, unsigned int x, unsigned int y, std::shared_ptr<nodeHybrid> parent, std::vector<int> neighbours) {
            this->id = id;
            this->x = x;
            this->y = y;
            this->parent = parent;
            this->yaw = 0.0;
            this->neighbours = neighbours;
            this->is_outside = outside;
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
            return x == other.x && y == other.y;
        }

        bool operator!=(const nodeHybrid & other) const
        {
            return !(*this == other);
        }

        
};

#endif // __nodeHybrid__hpp__
