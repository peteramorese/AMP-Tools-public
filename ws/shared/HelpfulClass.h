#pragma once
#include "AMPCore.h"
using std::vector, std::string, std::cout, Eigen::Vector2d;

class MyClass {
    public:
        amp::Polygon findMinkowskiDiff(const amp::Obstacle2D& obstacle, std::vector<Eigen::Vector2d> robotVertices);
        std::vector<amp::Polygon> findCSpaceObstacles(const amp::Obstacle2D& obstacle, std::vector<Eigen::Vector2d> robotVertices);
};

bool isPointInsidePolygon(const Vector2d& point, const vector<Vector2d>& polygon);

double distanceBetweenPoints(const Vector2d& point1, const Vector2d& point2);