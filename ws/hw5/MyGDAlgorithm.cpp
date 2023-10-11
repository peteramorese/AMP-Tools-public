#include "MyGDAlgorithm.h"
#include<iostream>
// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    currentXY = problem.q_init;
    path.waypoints.push_back(problem.q_init);
    const double epsilon = 0.25;
    const double alpha = 0.1;
    Eigen::Vector2d grad = getGradient(currentXY, problem);
    int steps = 0;
    std::cout << "grad " << grad << " gradnorm " << grad.norm() << std::endl;
    while((grad).norm() > epsilon && steps < 10000){
        std::cout << "here " << currentXY << std::endl;
        grad = getGradient(currentXY, problem);
        currentXY = currentXY - alpha*grad;
        path.waypoints.push_back(currentXY);
        steps++;
        std::cout << "now " << currentXY << std::endl;
    }


    path.waypoints.push_back(problem.q_goal);

    return path;
}