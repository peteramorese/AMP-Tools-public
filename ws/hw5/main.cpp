// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"
#include "hw/HW5.h"

// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

void problem1() {
    Problem2D problem = HW2::getWorkspace1();
    MyGDAlgorithm robot(1, 15, 1, 1, 0.1);
    Path2D path = robot.plan(problem);
    Visualizer::makeFigure(problem, path);
}

void problem2() {
    Problem2D problem = HW2::getWorkspace2();
    MyGDAlgorithm robot(1, 1, 1, 1, 1);
    Path2D path = robot.plan(problem);
    Visualizer::makeFigure(problem, path);
}

void problem3() {
    Problem2D problem = HW5::getWorkspace1();
    MyGDAlgorithm robot(0.1, 5, 1, 1, 0.05);
    Path2D path = robot.plan(problem);
    Visualizer::makeFigure(problem, path);
}

int main(int argc, char** argv) {
    problem3();

    Visualizer::showFigures();
    //HW2::grade<MyBugAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv);
    return 0;
}