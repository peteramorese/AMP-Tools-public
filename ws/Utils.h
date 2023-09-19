#pragma once

#include <Eigen/Core>
#include "AMPCore.h"
#include "hw/HW2.h"
#include "tools/Environment.h"

namespace Utils {
    bool checkStep(Eigen::Vector2d start, Eigen::Vector2d stop, const amp::Problem2D& problem);

    bool checkLineSegmentIntersect(Eigen::Vector2d start, Eigen::Vector2d stop,
        Eigen::Vector2d obsStart, Eigen::Vector2d obsStop);

    Eigen::Vector2d rotateVec(Eigen::Vector2d vector, double angle);
}