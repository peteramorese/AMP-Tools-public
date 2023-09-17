#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...
        std::vector<Eigen::Vector2d> returnToMinDist(Eigen::Vector2d minDist, Eigen::Vector2d robotPosition, amp::Path2D path) const;
        Eigen::Vector2d stepToGoal(const amp::Problem2D& problem, Eigen::Vector2d location, float delta) const;
        std::list<std::pair<Eigen::Vector2d, Eigen::Vector2d>> traverseOrder(std::vector<Eigen::Vector2d>& vertices, int destination1, Eigen::Vector2d intersectPosition) const;
        void stepLine(Eigen::Vector2d vert1, Eigen::Vector2d vert2, Eigen::Vector2d& step, float delta) const;
        bool occupied(Eigen::Vector2d location, const amp::Problem2D& problem, std::list<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& obstacleLines, float near) const;
        bool isPointOnLine(Eigen::Vector2d vert1, Eigen::Vector2d  vert2, Eigen::Vector2d  location, float near) const;
        std::list<Eigen::Vector2d> traverseObstacle(Eigen::Vector2d location, std::list<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& traverseList, float delta) const;
        Eigen::Vector2d stepLine2(Eigen::Vector2d vert1, Eigen::Vector2d vert2, float delta) const;
        float distance(Eigen::Vector2d vert1, Eigen::Vector2d vert2) const;
        Eigen::Vector2d computeCentroid(const std::vector<Eigen::Vector2d>& points) const;
        std::vector<Eigen::Vector2d> expandObstacle(amp::Obstacle2D obstacle, float delta) const;
        amp::Problem2D expandProblem(const amp::Problem2D& problem, float delta) const;

    private:
};