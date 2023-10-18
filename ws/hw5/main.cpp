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
    MyGDAlgorithm robot(0.1, 3, 0.05, 0.5, 0.5);
    Path2D path = robot.plan(problem);
    Visualizer::makeFigure(problem, path);
}

void problem2() {
    Problem2D problem = HW2::getWorkspace2();
    MyGDAlgorithm robot(0.1, 5, 0.5, 3, 0.5);
    Path2D path = robot.plan(problem);
    Visualizer::makeFigure(problem, path);
}

void problem3() {
    Problem2D problem = HW5::getWorkspace1();
    MyGDAlgorithm robot(0.1, 3, 0.05, 1, 0.5);
    Path2D path = robot.plan(problem);
    Visualizer::makeFigure(problem, path);
}

int main(int argc, char** argv) {
    problem1();
    // problem2();
    // problem3();
    // MyGDAlgorithm robot(0.1, 3, 0.05, 1, 0.5);
    Visualizer::showFigures();
    
    // HW5::grade<MyGDAlgorithm>("yusif.razzaq@colorado.edu", argc, argv, 0.1, 3, 0.05, 2, 0.5);
    return 0;
}