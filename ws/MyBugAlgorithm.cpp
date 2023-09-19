#include "MyBugAlgorithm.h"
#include "Bug1.h"
#include <vector>
#include <cmath>
#include "tools/Obstacle.h" 
#include "Line.h"
#include <list>
#include "Bug2.h"
#include "Bug1.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    float stepSize = 0.1;
    float delta = 0.1;

    // Bug 1 implementation:
    Bug1 bug1 = Bug1(problem, stepSize, delta);
    path.waypoints.push_back(problem.q_init);
    while(bug1.position != problem.q_goal){
        bug1.step();
        bug1.waypoints.push_back(bug1.position);
        path.waypoints.push_back(bug1.position);
    }

    //Bug 2 implementation:
    // Bug2 bug2 = Bug2(problem, stepSize, delta);
    // path.waypoints.push_back(problem.q_init);
    // while(bug2.position != problem.q_goal){
    //     bug2.step();
    //     bug2.waypoints.push_back(bug2.position);
    //     path.waypoints.push_back(bug2.position);
    // }

    float distance = 0;
    for(int i = 1; i < path.waypoints.size(); i++){
        distance += sqrt(pow(path.waypoints[i][0] - path.waypoints[i-1][0], 2) + pow(path.waypoints[i][1] - path.waypoints[i-1][1], 2));
    }
    std::cout << "Distance: " << distance << std::endl;

    return path;

}

