#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include "Utils.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file

        MyBugAlgorithm();
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...
        Eigen::Vector2d step(std::vector<Eigen::Vector2d> path, const amp::Problem2D& problem);
        Eigen::Vector2d stepBug2(std::vector<Eigen::Vector2d> path, const amp::Problem2D& problem);
        
    private:
        // Add any member variables here...
        int bugType = 2;
        int boundaryFollowing = 0;
        std::vector<Eigen::Vector2d> boundaryTrace;
        std::vector<double> boundaryDistances;
        Eigen::Vector2d hitPoint;
        Eigen::Vector2d curDir;
        const double stepSize = .1;
        const int maxSteps = 10000;
        Eigen::Vector2d curGoal;
};