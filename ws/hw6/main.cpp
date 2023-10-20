// // This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

#include "hw/HW2.h"
#include "hw/HW6.h"
#include "CSpaceConstructor.h"
#include "MyLinkManipulator.h"
#include "WaveFront.h"
#include "MyAStar.h"
#include "HelpfulClass.h"

using namespace amp;
using Eigen::Vector2d, std::vector, std::cout;

void problem1a() {
    Problem2D problem = HW2::getWorkspace1();
    MyPointWFAlgo algo;
    std::unique_ptr<amp::GridCSpace2D> cSpace = algo.constructDiscretizedWorkspace(problem);
    amp::Path2D path = algo.planInCSpace(problem.q_init, problem.q_goal, *cSpace);
    Visualizer::makeFigure(*cSpace);
    Visualizer::makeFigure(problem, path);
}

void problem1b() {
    Problem2D problem = HW2::getWorkspace2();
    MyPointWFAlgo algo;
    std::unique_ptr<amp::GridCSpace2D> cSpace = algo.constructDiscretizedWorkspace(problem);
    amp::Path2D path = algo.planInCSpace(problem.q_init, problem.q_goal, *cSpace);
    Visualizer::makeFigure(*cSpace);
    Visualizer::makeFigure(problem, path);
}

void problem2() {
    Problem2D problem = HW6::getHW4Problem1();
    MyLinkManipulator manipulator({1, 1});
    ManipulatorState initState = manipulator.getConfigurationFromIK(problem.q_init);
    ManipulatorState goalState = manipulator.getConfigurationFromIK(problem.q_goal);
    MyManipWFAlgo algo;
    std::unique_ptr<amp::GridCSpace2D> cSpace = algo.constructDiscretizedWorkspace(manipulator, problem);
    amp::Path2D path = algo.planInCSpace({initState[0], initState[1]}, {goalState[0], goalState[1]}, *cSpace);

    int numStates = path.waypoints.size();
    for (int i = 0; i < numStates; i += numStates/10) {
        Vector2d state = path.waypoints[i];
        Visualizer::makeFigure(problem, manipulator, {state(0), state(1)});
    }
    Visualizer::makeFigure(*cSpace);
}

void problem3() {
    ShortestPathProblem problem = HW6::getEx3SPP();
    LookupSearchHeuristic heuristic = HW6::getEx3Heuristic();
    MyAStarAlgo algo;
    MyAStarAlgo::GraphSearchResult result = algo.search(problem, heuristic);
}

// void problem3a() {
//     Environment2D workspace = HW4::getEx3Workspace1();
//     vector<double> linkLengths = {1, 1};
//     CSpaceConstructor cSpace(360, 360, -10, 10, -10, 10);
//     cSpace.populateGrid(linkLengths, workspace.obstacles);
//     Visualizer::makeFigure(workspace.obstacles);
// }



int main(int argc, char** argv) {
    // problem1a();
    // problem1b();
    // problem2();
    problem3();
    Visualizer::showFigures();
    return 0;
}