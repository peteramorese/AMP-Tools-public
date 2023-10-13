#include "MyGDAlgorithm.h"
#include<iostream>

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    currentXY = problem.q_init;
    path.waypoints.push_back(problem.q_init);
    const double epsilon = 0.25;
    const double alpha = 0.01;
    Eigen::Vector2d grad;
    int steps = 0;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-1.0,1.0);
    do{
        grad = getGradient(currentXY, problem);
        if((grad).norm() <= epsilon && (currentXY - problem.q_goal).norm() > 0.5 ){
            // while((grad).norm() <= epsilon && steps < 20000){
                grad(0) = distribution(generator);
                grad(1) = distribution(generator);
                steps++;
            // }
            // std::cout << "wiggling, new grad " << grad << std::endl;
        }
        currentXY = currentXY - alpha*grad;
        path.waypoints.push_back(currentXY);
        steps++;
        
    }while((grad).norm() > epsilon && steps < 20000);
    if(steps >= 20000){
        std::cout << "Took too long, bro is lost XD" << std::endl;
    }
    else{
        path.waypoints.push_back(problem.q_goal);
    }


    return path;
}