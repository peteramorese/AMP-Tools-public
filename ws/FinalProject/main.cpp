#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyFlightPlanner.h"
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());
    UASProblem prob(3,3,50,0.1,4.0,0.1,5,0.75);
    //UASProblem(n_GA, n_UAV, n_Obs, min_Obs, max_Obs, size_UAV, los_dist, conRad);
    
    MyFlightPlanner fPlanner;
    fPlanner.getN() = 200;
    fPlanner.makeFlightPlan(10,3,prob);
    
    Visualizer::showFigures();
}