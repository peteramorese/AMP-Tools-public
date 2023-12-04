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
    problem.print();
    MyPointWFAlgo algo;
    std::unique_ptr<amp::GridCSpace2D> cSpace = algo.constructDiscretizedWorkspace(problem);
    amp::Path2D path = algo.planInCSpace(problem.q_init, problem.q_goal, *cSpace);
    cout << path.length() << " = Path length\n";
    Visualizer::makeFigure(*cSpace);
    Visualizer::makeFigure(problem, path);
}

void problem1b() {
    Problem2D problem = HW2::getWorkspace2();
    MyPointWFAlgo algo;
    std::unique_ptr<amp::GridCSpace2D> cSpace = algo.constructDiscretizedWorkspace(problem);
    amp::Path2D path = algo.planInCSpace(problem.q_init, problem.q_goal, *cSpace);
    cout << path.length() << "Path length\n";
    // Visualizer::makeFigure(*cSpace);
    Visualizer::makeFigure(problem, path);
}

void problem2() {
    // Problem2D problem = HW6::getHW4Problem1();
    // Problem2D problem = HW6::getHW4Problem2();
    Problem2D problem = HW6::getHW4Problem3(); 
    MyLinkManipulator manipulator({1, 1});
    ManipulatorState initState = manipulator.getConfigurationFromIK(problem.q_init);
    ManipulatorState goalState = manipulator.getConfigurationFromIK(problem.q_goal);
    MyManipWFAlgo algo;
    std::unique_ptr<amp::GridCSpace2D> cSpace = algo.constructDiscretizedWorkspace(manipulator, problem);
    amp::Path2D path = algo.planInCSpace({initState[0], initState[1]}, {goalState[0], goalState[1]}, *cSpace);

    Visualizer::makeFigure(problem, manipulator, path);
    Visualizer::makeFigure(*cSpace, path);
}

void problem3a() {
    ShortestPathProblem problem = HW6::getEx3SPP();
    LookupSearchHeuristic heuristic = HW6::getEx3Heuristic();
    MyAStarAlgo algo(false);
    MyAStarAlgo::GraphSearchResult result = algo.search(problem, heuristic);
}

void problem3b() {
    ShortestPathProblem problem = HW6::getEx3SPP();
    LookupSearchHeuristic heuristic;
    MyAStarAlgo algo(true);
    MyAStarAlgo::GraphSearchResult result = algo.search(problem, heuristic);
}

int main(int argc, char** argv) {
    // problem1a();
    // problem1b();
    problem2();
    // problem3a();
    // problem3b();
    Visualizer::showFigures();
    // amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("yusif.razzaq@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(), std::make_tuple(false));
    return 0;
}