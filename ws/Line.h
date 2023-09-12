#pragma once
#include <cmath>
#include <vector>
#include "AMPCore.h"
#include "hw/HW2.h"

class Line {
    public:

        Line(Eigen::Vector2d p1, Eigen::Vector2d p2);
        ~Line();

        Eigen::Vector2d point1;
        Eigen::Vector2d point2;
        float m;

};