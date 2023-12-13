#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyFlightPlanner.h"
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());
    UASProblem prob;
    

    MyFlightPlanner fPlanner;
    Visualizer::makeFigure(prob,prob.GApaths);
    Visualizer::showFigures();
    // MultiAgentPath2D soln =  fPlanner.plan(prob);

    return 0;
}