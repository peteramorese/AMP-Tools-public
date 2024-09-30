#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
//ZACK what does it mean to "derive" the bug algorithm earlier? does it inherit something from that class?
class MyBugAlgorithm : public amp::BugAlgorithm { //ZACK what does the public part mean?
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        bool inCollision(Eigen::Vector2d position, std::vector<std::vector<int>> obs_primitives, const amp::Problem2D& problem);
        bool PointInPolygon(Eigen::Vector2d point, const amp::Problem2D& problem);
        amp::Path2D followObstacle(Eigen::Vector2d point, const amp::Problem2D& problem, amp::Path2D path);
        double Radians(double degrees);
        double DistToGoal(Eigen::Vector2d point1, const amp::Problem2D& problem);
        double Dist(Eigen::Vector2d point1, Eigen::Vector2d point2);
        bool PointInMLine(Eigen::Vector2d point, const amp::Problem2D& problem);
        amp::Obstacle2D CollisionWithObs(Eigen::Vector2d point, const amp::Problem2D& problem);
        amp::Path2D followObs2(Eigen::Vector2d point, const amp::Problem2D& problem, amp::Path2D path, amp::Obstacle2D obs);


        //make sure all virtual methods stay as is because they are coming from the toolbox

        // Add any other methods here...

    
    private:
        // Add any member variables here...
        //obstacles; //added with yusif
        //std::vector<std::vector<int>> obs_primitives;

};

