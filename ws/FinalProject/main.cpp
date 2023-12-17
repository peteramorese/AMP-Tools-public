#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyFlightPlanner.h"
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());
    UASProblem prob(2,3,70,0.1,4.0,0.1,4,0.75); //UASProblem(n_GA, n_UAV, n_Obs, min_Obs, max_Obs, size_UAV, los_dist, conRad)
    // UASProblem prob(3,3,20,0.75,4.0,0.1,7,0.75);
    MyFlightPlanner fPlanner;
    fPlanner.getN() = 200; //Max number of attempts for finding the next centralized planner state
    fPlanner.makeFlightPlan(1,7,6,prob); //(min number of UAVs,max number of UAVs, max attempts to solve with n UAVs, problem)
    // fPlanner.kino = true;
    // fPlanner.makeFlightPlan(5,8,prob);
    Visualizer::showFigures();
}