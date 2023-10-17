// // This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// // Include the correct homework header
#include "hw/HW6.h"
#include "WaveFront.h"

// // Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;
using Eigen::Vector2d, std::vector, std::cout;

void problem1() {
    Problem2D problem = HW6::getHW4Problem1();
    MyPointWFAlgo algo;
    std::unique_ptr<amp::GridCSpace2D> cSpace = algo.constructDiscretizedWorkspace(problem);
    amp::Path2D path = algo.planInCSpace(problem.q_init, problem.q_goal, *cSpace);
    // Visualizer::makeFigure(manipulator, state);
}

// void problem2a() {
//     vector<double> linkLengths = {0.5, 1, 0.5};
//     ManipulatorState state = {M_PI/6, M_PI/3, 7*M_PI/4};
//     MyLinkManipulator manipulator(linkLengths);
//     Visualizer::makeFigure(manipulator, state);
// }

// void problem2b() {
//     Vector2d endEffector(2, 0);
//     vector<double> linkLengths = {1, 0.5, 1};
//     MyLinkManipulator manipulator(linkLengths);
//     ManipulatorState state = manipulator.getConfigurationFromIK(endEffector);
//     Visualizer::makeFigure(manipulator, state);
// }

// void problem3a() {
//     Environment2D workspace = HW4::getEx3Workspace1();
//     vector<double> linkLengths = {1, 1};
//     CSpaceConstructor cSpace(360, 360, -10, 10, -10, 10);
//     cSpace.populateGrid(linkLengths, workspace.obstacles);
//     Visualizer::makeFigure(cSpace);
//     Visualizer::makeFigure(workspace.obstacles);
// }

// void problem3b() {
//     Environment2D workspace = HW4::getEx3Workspace2();
//     vector<double> linkLengths = {1, 1};
//     CSpaceConstructor cSpace(360, 360, -10, 10, -10, 10);
//     cSpace.populateGrid(linkLengths, workspace.obstacles);
//     Visualizer::makeFigure(cSpace);
//     Visualizer::makeFigure(workspace.obstacles);
// }

// void problem3c() {
//     Environment2D workspace = HW4::getEx3Workspace3();
//     // for (const amp::Obstacle2D& obstacle : workspace.obstacles) {
//     //     for (const Vector2d& vertex : obstacle.verticesCW()) {
//     //         cout << "Vertex (" << vertex(0) << ", " << vertex(1) << ")\n";
//     //     }
//     // }
//     vector<double> linkLengths = {1, 1};
//     CSpaceConstructor cSpace(360, 360, -10, 10, -10, 10);
//     cSpace.populateGrid(linkLengths, workspace.obstacles);
//     Visualizer::makeFigure(cSpace);
//     Visualizer::makeFigure(workspace.obstacles);
// }

int main(int argc, char** argv) {
    problem1();
    return 0;
}