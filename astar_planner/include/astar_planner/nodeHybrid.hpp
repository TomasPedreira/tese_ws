#ifndef __nodeHybrid__hpp__
#define __nodeHybrid__hpp__


#include <memory>
#include <vector>

class nodeHybrid {
    public:
        unsigned int x, y;
        unsigned int tx, ty;
        int id;
        double f, g, h, yaw;
        std::shared_ptr<nodeHybrid> parent;
        std::vector<int> neighbours;
        bool is_outside;
        double trailer_yaw;
        bool is_dubins, is_hybrid, is_voronoi;

        nodeHybrid() {
            x = 0;
            y = 0;
            tx = 0;
            ty = 0;
            id = 0;
            f = 0.0;
            g = 0.0;
            h = 0.0;
            yaw = 0.0;
            parent = nullptr;
            neighbours.clear();
            is_outside = false;
            trailer_yaw = 0.0;
            is_dubins = false;
            is_hybrid = false;
            is_voronoi = false;
        };

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
