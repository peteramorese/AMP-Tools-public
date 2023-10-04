// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"
#include "MyLinkManipulator.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;
using Eigen::Vector2d, std::vector, std::cout;

int main(int argc, char** argv) {
    Obstacle2D obstacle1 = HW4::getEx1TriangleObstacle();
    vector<Vector2d> robotVertices = { Vector2d(0, 0) , Vector2d(1, 2), Vector2d(0, 2) };
    MyClass myClass;
    // Polygon poly = myClass.findMinkowskiDiff(obstacle1, robotVertices);
    // vector<double> angles;
    // for (int i = 0; i < 12; ++i) {
    //     angles.push_back(2*M_PI / 12 * i);
    // }
    vector<Polygon> polygons = myClass.findCSpaceObstacles(obstacle1, robotVertices);
    Visualizer::makeFigure(polygons, angles);

    vector<double> linkLengths = {1.0, 1.0};
    ManipulatorState state = {0.0, 3.14/2};
    MyLinkManipulator manipulator(linkLengths);

    Visualizer::makeFigure(manipulator, state);
    Visualizer::showFigures();


    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    // amp::RNG::seed(amp::RNG::randiUnbounded());

    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "nonhuman.biologic@myspace.edu", argc, argv);
    return 0;
}