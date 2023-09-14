#include "MyBugAlgorithm.h"
#include "Bug.h"
#include <vector>
#include <cmath>
#include "tools/Obstacle.h" 
#include "Line.h"
#include <list>

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) const {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    float delta = 0.1;

    //step 1: expand all boundries of obstacles by delta;
    amp::Problem2D expandedProblem = expandProblem(problem, .1);

    //step 2: initialize bug
    Bug bug = Bug(expandedProblem, delta);

    path.waypoints.push_back(problem.q_init);
    for(int i = 0; i < 200; i++){
        if(i == 188){
            std::cout << "break" << std::endl;
        }
        bug.step();
        bug.waypoints.push_back(bug.position);
        path.waypoints.push_back(bug.position);
        // std::cout << "mode: " << bug.mode << std::endl;
        // for(Eigen::Vector2d point : bug.goalQueue){
        //     std::cout << point[0] << ", " <<point[1] << "-";
        // }
        // std::cout << std::endl;
    }
    return path;

}

/**
 * @brief Takes a step in direct direction towards vert2.
 * 
 * @param vert1 From point
 * @param vert2 To point
 * @return returnType Description of return value
 **/
Eigen::Vector2d MyBugAlgorithm::stepLine2(Eigen::Vector2d vert1, Eigen::Vector2d vert2, float delta) const{
    // Handle vertical line case where m = inf:
    if (abs(vert1[0] - vert2[0]) < delta) {
       if (vert2[1] > vert1[1]){
            return Eigen::Vector2d(vert1[0], vert1[1] + delta);
       }
       else{
            return Eigen::Vector2d(vert1[0], vert1[1] - delta);
       }
    }
    float m = (vert2[1] - vert1[1]) / (vert2[0] - vert1[0]);
    float stepX = ( vert2[0] > vert1[0] ) ? vert1[0] + delta : vert1[0] - delta;
    float stepY = m * (stepX - vert1[0]) + vert1[1];
    return Eigen::Vector2d (stepX, stepY);
}

/**
 * @brief Describe the purpose of the function here
 * 
 * @param paramName Description of parameter
 * @return returnType Description of return value
 **/
Eigen::Vector2d MyBugAlgorithm::computeCentroid(const std::vector<Eigen::Vector2d>& points) const{
    Eigen::Vector2d centroid(0, 0);

    if (points.empty()) {
        return centroid;  // Return (0,0) if no points are provided.
    }

    for (const auto& point : points) {
        centroid += point;
    }

    return centroid / static_cast<double>(points.size());
}

/**
 * @brief Describe the purpose of the function here
 * 
 * @param paramName Description of parameter
 * @return returnType Description of return value
 **/
std::vector<Eigen::Vector2d> MyBugAlgorithm::expandObstacle(amp::Obstacle2D obstacle, float delta) const{
    std::vector<Eigen::Vector2d> enlargedObstacle;
    std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
    Eigen::Vector2d centroid = computeCentroid(vertices);
    for (Eigen::Vector2d vertex : vertices){
        Eigen::Vector2d enlargedVertex;
        if (vertex[0] < centroid[0]){
            enlargedVertex[0] = vertex[0] - delta;
        }
        else{
            enlargedVertex[0] = vertex[0] + delta;
        }
        if (vertex[1] < centroid[1]){
            enlargedVertex[1] = vertex[1] - delta;
        }
        else{
            enlargedVertex[1] = vertex[1] + delta;
        }
        enlargedObstacle.push_back(enlargedVertex);
    }
    return enlargedObstacle;
}

/**
 * @brief creates new problem where all obstacles are delta larger.
 * 
 * @param paramName Description of parameter
 * @return returnType Description of return value
 **/
amp::Problem2D MyBugAlgorithm::expandProblem(const amp::Problem2D& problem, float delta) const{
    amp::Problem2D expandedProblem;
    expandedProblem.q_init = problem.q_init;
    expandedProblem.q_goal = problem.q_goal;
    expandedProblem.obstacles = problem.obstacles;
    expandedProblem.obstacles.clear();
    for(amp::Obstacle2D obstacle : problem.obstacles){
        std::vector<Eigen::Vector2d> enlargedObstacle = expandObstacle(obstacle, delta);
        amp::Obstacle2D enlargedObstacle2D(enlargedObstacle);
        expandedProblem.obstacles.push_back(enlargedObstacle2D);
    }
    return expandedProblem;
}

