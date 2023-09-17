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
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) const {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    float stepSize = 0.1;
    float delta = 0.1;

    //step 1: initialize bug
    Bug2 bug2 = Bug2(problem, stepSize, delta);

    path.waypoints.push_back(problem.q_init);
    while(bug2.position != problem.q_goal){
        bug2.step();
        bug2.waypoints.push_back(bug2.position);
        path.waypoints.push_back(bug2.position);
    }
    return path;

}

