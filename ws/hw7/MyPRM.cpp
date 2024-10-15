#include "MyPRM.h"


namespace amp{
    amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {    
        amp::Path2D path;
        path.waypoints.push_back(problem.q_init);
        path.waypoints.push_back(problem.q_goal);
        return path;
    }
}
