#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

class Bug {
    public:
        // methods:
        Bug(const amp::Problem2D& p, float sZ);
        ~Bug();

        void step();

        // fields
        amp::Problem2D environment;
        std::list<Eigen::Vector2d> goalQueue;
        Eigen::Vector2d hitPoint;
        Eigen::Vector2d position;
        float stepSize;
        int mode;
        float minDist;
        Eigen::Vector2d minDistPoint;
        std::vector<Eigen::Vector2d> waypoints;
};