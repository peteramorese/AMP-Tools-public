#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

class Helper {
    public:
        // methods:
        Helper();
        ~Helper();

        bool isPointOnSegment(const Eigen::Vector2d &start, const Eigen::Vector2d &end, const Eigen::Vector2d &point);
        std::list<Eigen::Vector2d> getObstacleTraverseVertices(std::vector<Eigen::Vector2d>& vertices, int destination1, Eigen::Vector2d position);
        std::list<Eigen::Vector2d> shortestPath(Eigen::Vector2d position, Eigen::Vector2d problem, std::vector<Eigen::Vector2d> waypoints);
        float getDistance(Eigen::Vector2d point1, Eigen::Vector2d point2);
        Eigen::Vector2d minDistance(Eigen::Vector2d point, std::vector<Eigen::Vector2d> waypoints);
        Eigen::Vector2d shortestVectorDist(Eigen::Vector2d vert1, Eigen::Vector2d vert2, Eigen::Vector2d point);
        Eigen::Vector2d computeCentroid(const std::vector<Eigen::Vector2d>& points) const;
        std::vector<Eigen::Vector2d> expandObstacle(amp::Obstacle2D obstacle, float delta) const;
        Eigen::Vector2d expandVertex(Eigen::Vector2d vert1, Eigen::Vector2d vert2, Eigen::Vector2d vert3, Eigen::Vector2d vert4);
        bool close(Eigen::Vector2d goal, Eigen::Vector2d position, float delta);

        // fields
};