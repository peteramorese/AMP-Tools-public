#include "MyFlightPlanner.h"


using Node = uint32_t;

amp::MultiAgentPath2D MyFlightPlanner::plan(const amp::MultiAgentProblem2D& problem){
    MyGoalBiasRRTND RRT;
    RRT.getN() = 50000;
    RRT.getS() = 0.5;
    // RRT.getN() = 7500;
    // RRT.getS() = 0.5;
    RRT.getG() = 0.15;
    // RRT.getE() = 0.25;
    auto start = std::chrono::high_resolution_clock::now();








    auto graph = std::make_shared<amp::Graph<double>>();
    amp::MultiAgentPath2D soln = RRT.plan(problem);













    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::milliseconds>(stop - start);
    time = duration.count();
    numIterations = RRT.getN();
    return soln;
}

