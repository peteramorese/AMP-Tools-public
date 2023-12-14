#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyFlightPlanner.h"
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());
    UASProblem prob(5,3,10,2.0,4.0);
    

    MyFlightPlanner fPlanner;
    // Visualizer::makeFigure(prob,prob.GApaths);
    // for(auto ln : prob.endGAt){
    //     LOG(ln);
    // }
    MultiAgentPath2D soln =  fPlanner.plan(prob);
    Visualizer::makeFigure(prob,soln);
    Visualizer::showFigures();
    return 0;
}