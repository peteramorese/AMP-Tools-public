#include "MyGDAlgorithm.h"
#include<iostream>
// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(problem.q_goal);
    return path;
}