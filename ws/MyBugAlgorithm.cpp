#include "MyBugAlgorithm.h"
#include <vector>
#include <cmath>
#include "tools/Obstacle.h" 
#include "Line.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) const {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    Eigen::Vector2d step; // robot's next potential position
    Eigen::Vector2d robotPosition(problem.q_init[0] , problem.q_init[1]); // robot's current position
    amp::Obstacle2D* hitObstaclePtr;
    Eigen::Vector2d* vert1Ptr;
    Eigen:: Vector2d* vert2Ptr;
    Eigen::Vector2d qShortest;
    float delta = 0.01;

    path.waypoints.push_back(robotPosition);

    // head towards goal:
    step = stepToGoal(problem, robotPosition, delta);
    while ( !occupied(step, problem, hitObstaclePtr, vert1Ptr, vert2Ptr)){
        robotPosition = step;
        step = stepToGoal(problem, step, delta);
    }
    step = robotPosition;
    path.waypoints.push_back(robotPosition);

    // follow obstacle boundry:
    

    return path;
}

Eigen::Vector2d MyBugAlgorithm::stepLine(Eigen::Vector2d* vert1Ptr, Eigen::Vector2d* vert2Ptr, Eigen::Vector2d step, float delta) const {
    
    // Handle vertical line case:
    if ((*vert1Ptr)[0] == (*vert2Ptr)[0]) {
        if ((*vert1Ptr)[0] > step[1]) {
            return Eigen::Vector2d (step[0], step[0] - delta);
        } else {
            return Eigen::Vector2d (step[0], step[0] + delta);
        }
    }

    float m = ((*vert2Ptr)[1] - (*vert1Ptr)[1]) / ((*vert2Ptr)[0] - (*vert1Ptr)[0]);
    float stepX = ( (*vert1Ptr)[0] > (*vert2Ptr)[0] ) ? step[0] - delta : step[0] + delta;
    float stepY = m * (stepX - step[0]) + step[1];
    return Eigen::Vector2d (stepX, stepY);
}

Eigen::Vector2d MyBugAlgorithm::stepToGoal(const amp::Problem2D& problem, Eigen::Vector2d location, float delta) const {
    // Handle vertical line case:
    if (location[0] == problem.q_goal[0]) {
        if (location[0] > problem.q_goal[1]) {
            return Eigen::Vector2d (location[0], location[0] - delta);
        } else {
            return Eigen::Vector2d (location[0], location[0] + delta);
        }
    }

    float m = (problem.q_goal[1] - location[1]) / (problem.q_goal[0] - location[0]);
    float stepX = ( problem.q_goal[0] > location[0] ) ? location[0] + delta : location[0] - delta;
    float stepY = m * (stepX - location[0]) + location[1];
    return Eigen::Vector2d (stepX, stepY);
}

bool MyBugAlgorithm::occupied(Eigen::Vector2d location, const amp::Problem2D& problem, amp::Obstacle2D* hitObstaclePtr, Eigen::Vector2d* vert1Ptr, Eigen::Vector2d* vert2Ptr) const {
    // given a robot location (x,y) check to see if the location is occupied by an obstacle.
    bool isOccupied = false;
    for (amp::Obstacle2D obstacle : problem.obstacles){
        std::vector<Eigen::Vector2d>& vertices = obstacle.verticesCCW();
        for (size_t i = 0; i < vertices.size(); ++i){
            if (i == vertices.size() - 1) {
                if (isPointOnLine(vertices[i], vertices[0], location)) {
                    hitObstaclePtr = &obstacle;
                    vert2Ptr = &vertices[i];
                    vert1Ptr = &vertices[0];
                    return true;
                }
            }
            else{
                if (isPointOnLine(vertices[i], vertices[i+1], location)) {
                    hitObstaclePtr = &obstacle;
                    vert2Ptr = &vertices[i];
                    vert1Ptr = &vertices[i+1];
                    return true;
                }
            }
        }
    }
    return false;
}

bool MyBugAlgorithm::isPointOnLine(Eigen::Vector2d vert1, Eigen::Vector2d vert2, Eigen::Vector2d location) const {
    // Given two vertices, check to see if the robot location is on the line between them.
    if (vert1[0] == vert2[0]) {
        return location[0] == vert1[0];
    }
    float m = (vert2[1] - vert1[1]) / (vert2[0] - vert1[0]);
    float epsilon = 1e-6;
    return std::abs(location[1] - vert1[1] - m * (location[1] - vert1[0])) < epsilon;
}
