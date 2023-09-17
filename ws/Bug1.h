#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

class Bug1 {
    public:
        // methods:
        Bug1(const amp::Problem2D& p, float sZ, float d);
        ~Bug1();

        void step();

        // fields
        amp::Problem2D environment;
        std::list<Eigen::Vector2d> goalQueue;
        Eigen::Vector2d hitPoint;
        Eigen::Vector2d leavePoint;
        Eigen::Vector2d position;
        float stepSize;
        float delta;
        int mode;
        float minDist;
        Eigen::Vector2d minDistPoint;
        std::vector<Eigen::Vector2d> waypoints;
};