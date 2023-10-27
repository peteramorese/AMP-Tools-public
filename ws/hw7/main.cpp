// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW6.h"
#include "MyPRM.h"

using namespace amp;

void problem1a() {
    Problem2D problem = HW5::getWorkspace1();
    MyPRM algo(200, 2);
    Path2D path = algo.plan(problem);
    cout << "Path length: " << path.length() << "\n";
    Visualizer::makeFigure(problem, path);
}

void problem1b() {
    Problem2D problem = HW2::getWorkspace1();
    // Problem2D problem = HW2::getWorkspace2();
    MyPRM algo(500, 2);
    Path2D path = algo.plan(problem);
    cout << "Path length: " << path.length() << "\n";
    Visualizer::makeFigure(problem, path);
}

void problem2() {
    // Problem2D problem = HW5::getWorkspace1();
    Problem2D problem = HW2::getWorkspace1();
    // Problem2D problem = HW2::getWorkspace2();
    MyRRT algo(500, 2);
    Path2D path = algo.plan(problem);
    cout << "Path length: " << path.length() << "\n";
    Visualizer::makeFigure(problem, path);
}


int main(int argc, char** argv) {
    // HW7::hint();
    // problem1a();
    problem1b();
    problem2();
    Visualizer::showFigures();
    // HW5::grade<MyGDAlgorithm>("yusif.razzaq@colorado.edu", argc, argv, 0.1, 3, 0.05, 2, 0.5);
    return 0;
}