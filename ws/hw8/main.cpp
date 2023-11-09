// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"
#include "MyRRT.h"

using namespace amp;

void problem1() {
    MultiAgentProblem2D problem = HW8::getWorkspace1(3);
    MyCentralPlanner planner(1000, 1.0, 0.05);
    MultiAgentPath2D path = planner.plan(problem);
    Visualizer::makeFigure(problem, path);
    Visualizer::showFigures();
    // HW8::generateAndCheck(planner);
}

void problem2() {
    MultiAgentProblem2D problem = HW8::getWorkspace1();
    Eigen::VectorXd v(2);
    v << 9.12871, 5.4625;
    MyCentralChecker collisionChecker(problem);
    bool tf = collisionChecker.inCollision(v);
    cout << tf << std::endl;
    Visualizer::makeFigure(problem);
    Visualizer::showFigures();
    }

int main(int argc, char** argv) {
    problem1();
    // problem2();

    // Visualizer::showFigures();
    // HW7::grade<MyPRM, MyRRT>("yusif.razzaq@colorado.edu", argc, argv, std::make_tuple(500, 2, true), std::make_tuple(1000, 2, true));
    return 0;
}