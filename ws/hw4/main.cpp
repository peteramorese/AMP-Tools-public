// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"
#include "MyLinkManipulator.h"
#include "CSpaceConstructor.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;
using Eigen::Vector2d, std::vector, std::cout;

void problem1() {
    Obstacle2D obstacle1 = HW4::getEx1TriangleObstacle();
    vector<Vector2d> robotVertices = { Vector2d(0, 0) , Vector2d(1, 2), Vector2d(0, 2) };
    MyClass myClass;
    Polygon poly = myClass.findMinkowskiDiff(obstacle1, robotVertices);
    vector<double> angles;
    for (int i = 0; i < 12; ++i) {angles.push_back(2*M_PI / 12 * i);}
    vector<Polygon> polygons = myClass.findCSpaceObstacles(obstacle1, robotVertices);
    Visualizer::makeFigure({polygons[0]});
    Visualizer::makeFigure(polygons, angles);
}

void problem2a() {
    vector<double> linkLengths = {0.5, 1, 0.5};
    ManipulatorState state = {M_PI/6, M_PI/3, 7*M_PI/4};
    MyLinkManipulator manipulator(linkLengths);
    Visualizer::makeFigure(manipulator, state);
}

void problem2b() {
    Vector2d endEffector(2, 0);
    vector<double> linkLengths = {1, 0.5, 1};
    MyLinkManipulator manipulator(linkLengths);
    ManipulatorState state = manipulator.getConfigurationFromIK(endEffector);
    Visualizer::makeFigure(manipulator, state);
}

void problem3a() {
    Environment2D workspace = HW4::getEx3Workspace1();
    vector<double> linkLengths = {1, 1};
    CSpaceConstructor cSpace(100, 100, -10, 10, -10, 10);
    cSpace.populateGrid(linkLengths, workspace.obstacles);
    Visualizer::makeFigure(cSpace);
    Visualizer::makeFigure(workspace.obstacles);
}

void problem3b() {
    Environment2D workspace = HW4::getEx3Workspace2();
    vector<double> linkLengths = {1, 1};
    CSpaceConstructor cSpace(50, 50, -10, 10, -10, 10);
    cSpace.populateGrid(linkLengths, workspace.obstacles);
    Visualizer::makeFigure(cSpace);
    Visualizer::makeFigure(workspace.obstacles);
}

void problem3c() {
    Environment2D workspace = HW4::getEx3Workspace3();
    // for (const amp::Obstacle2D& obstacle : workspace.obstacles) {
    //     for (const Vector2d& vertex : obstacle.verticesCW()) {
    //         cout << "Vertex (" << vertex(0) << ", " << vertex(1) << ")\n";
    //     }
    // }
    vector<double> linkLengths = {1, 1};
    CSpaceConstructor cSpace(50, 50, -10, 10, -10, 10);
    cSpace.populateGrid(linkLengths, workspace.obstacles);
    Visualizer::makeFigure(cSpace);
    Visualizer::makeFigure(workspace.obstacles);
}

int main(int argc, char** argv) {
    // problem1();
    // problem2a();
    // problem2b();
    problem3a();
    // problem3b();
    // problem3c();
    Visualizer::showFigures();
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    // amp::RNG::seed(amp::RNG::randiUnbounded());
    // Grade method
    // vector<double> linkLengths = {1, 0.5, 1};
    // MyLinkManipulator manipulator(linkLengths);
    // CSpaceConstructor cSpace(360, 360, -10, 10, -10, 10);
    // amp::HW4::grade<MyLinkManipulator>(cSpace, "yusif.razzaq@colorado.edu", argc, argv);
    return 0;
}