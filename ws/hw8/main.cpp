#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiPlanner.h"
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    MyCentralizedMultiAgentRRT cenRRT;
    amp::MultiAgentProblem2D prob = HW8::getWorkspace1(3);
    amp::MultiAgentPath2D path = cenRRT.plan(prob);
    std::vector<std::vector<Eigen::Vector2d>> collision_states;
    HW8::check(path, prob, collision_states);
    Visualizer::makeFigure(prob,path, collision_states);

    MyDecentralizedMultiAgentRRT decRRT;
    prob = HW8::getWorkspace1(2);
    do{
        path = decRRT.plan(prob);
        collision_states.clear();
    }while(HW8::check(path, prob, collision_states));
    // HW8::check(path, prob, collision_states);
    Visualizer::makeFigure(prob,path, collision_states);
    Visualizer::makeFigure(prob,path);
    Visualizer::showFigures();

    // MyDecentralizedMultiAgentRRT decRRT;
    // HW8::grade(cenRRT, decRRT, "collin.hudson@colorado.edu", argc, argv);
    return 0;
}