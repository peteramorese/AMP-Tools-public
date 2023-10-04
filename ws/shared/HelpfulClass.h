#include "AMPCore.h"

class MyClass {
    public:
        amp::Polygon findMinkowskiDiff(const amp::Obstacle2D& obstacle, std::vector<Eigen::Vector2d> robotVertices);
        std::vector<amp::Polygon> findCSpaceObstacles(const amp::Obstacle2D& obstacle, std::vector<Eigen::Vector2d> robotVertices);

};
