#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiPlanner.h"
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    MyCentralizedMultiAgentRRT cenRRT;
    amp::MultiAgentProblem2D cenProb = HW8::getWorkspace1(6);
    amp::MultiAgentPath2D cenPath = cenRRT.plan(cenProb);
    std::vector<std::vector<Eigen::Vector2d>> collision_states;
    HW8::check(cenPath, cenProb, collision_states);
    Visualizer::makeFigure(cenProb,cenPath, collision_states);
    Visualizer::showFigures();

    MyDecentralizedMultiAgentRRT decRRT;
    HW8::grade(cenRRT, decRRT, "collin.hudson@colorado.edu", argc, argv);
    return 0;
}