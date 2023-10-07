// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "MyLinkManipulator.h"
#include "MyConfigurationSpace.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    std::vector<amp::Polygon> prob1;
    MyConfigEnvironment space1;
    amp::Polygon tempRobot;
    std::vector<double> height;
    for(int j = 0; j < 12; j++){
        tempRobot = HW4::getEx1TriangleObstacle();
        space1.rotateRobot(tempRobot.verticesCCW(),M_PI*(j/6.0));
        prob1.push_back(space1.getCspaceObs(tempRobot,HW4::getEx1TriangleObstacle()));
        if(j == 0){
        }
        height.push_back(M_PI*(j/6.0));
    }
    Visualizer::makeFigure(prob1,height);

    // MyLinkManipulator mani3;
    // std::vector<double> state(2,0.0);
    // amp::Environment2D env3 = HW4::getEx3Workspace1();
    // Visualizer::makeFigure(env3,mani3,state);
    // MyGridCSpace2DConstructor GridBldr;

    // const std::unique_ptr<amp::GridCSpace2D> ptr = GridBldr.construct(mani3,env3);
    // HW4::checkCSpace(*ptr, mani3, env3);
    // MyGridCSpace2D plotEnv3(250,250,0,2*M_PI,0,2*M_PI);
    // Visualizer::makeFigure(plotEnv3.makeCSpace(mani3,env3));

    // MyGridCSpace2DConstructor(mani3,env3);

    // Visualizer::makeFigure(mani2, reverseState);
    Visualizer::showFigures();

    // HW4::checkFK(mani2.getJointLocation(reverseState,2),2,mani2,reverseState,true);
    // Grade method
    // amp::HW4::grade<MyLinkManipulator>(GridBldr, "collin.hudson@colorado.edu", argc, argv);
    return 0;
}