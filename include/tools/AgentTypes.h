#pragma once

#include <Eigen/Core>

#include "tools/Obstacle.h" 
#include "tools/LinkManipulator.h" 

namespace amp {

/// @brief Point agent in 2D plane world represented by an `x` and `y`. No specific properties exist for a point agent (empty struct)
struct PointAgent2D {};

/// @brief Polygon agent in 2D plane world where the origin (0,0) in the body's frame is the reference point
struct PolygonAgent2D {
    amp::Polygon body;
};

/// @brief Circular disk agent in 2D plane world. Reference point is the center of the circle
struct DiskAgent2D {
    double radius;
};


}