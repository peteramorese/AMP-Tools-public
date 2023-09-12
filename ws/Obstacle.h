#pragma once
#include <vector>
#include "Line.h"


class Obstacle {
    public:

        Obstacle();
        ~Obstacle();

        std::vector<Line> obstacleBorders();

};