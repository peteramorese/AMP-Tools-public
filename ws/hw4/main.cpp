// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;
using Eigen::Vector2d;

int main(int argc, char** argv) {
    amp::Obstacle2D problem1 = HW4::getEx1TriangleObstacle();
    MyClass myClass;
    std::vector<Vector2d> robotVertices = { Vector2d(0, 0) , Vector2d(1, 2), Vector2d(0, 2) };

    myClass.findMinkowskiDiff(problem1, robotVertices);
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    // amp::RNG::seed(amp::RNG::randiUnbounded());

    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "nonhuman.biologic@myspace.edu", argc, argv);
    return 0;
}