#include "AMPCore.h"
#include "hw/HW4.h"
#include "hw/HW6.h"
#include "MyLinkManipulator.h"
#include "MyConfigurationSpace.h"
// #include "MyGridCSpace.h"
#include "MyAlgos.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());
    // MyManipWFAlgo mani;
    LOG("Problem 1");



    LOG("Problem 2");
    MyGridCSpace2DConstructor cons;
    cons.getGridWidth() = 0.25;
    MyManipWFAlgo mani(cons);
    MyLinkManipulator mani3;
    const amp::Problem2D env3 = HW6::getHW4Problem3();
    auto path = mani.plan(mani3,env3);
    MyGridCSpace2D plotEnv3(std::ceil((cons.getX0_bounds().second - cons.getX0_bounds().first)/cons.getGridWidth()),std::ceil((cons.getX1_bounds().second - cons.getX1_bounds().first)/cons.getGridWidth()),cons.getX0_bounds().first,cons.getX0_bounds().second,cons.getX1_bounds().first,cons.getX1_bounds().second);
    Visualizer::makeFigure(plotEnv3.makeCSpace(mani3,env3),path);

    Eigen::Vector2d lowerB(cons.getX0_bounds().first,cons.getX1_bounds().first);
    Eigen::Vector2d upperB(cons.getX0_bounds().second,cons.getX1_bounds().second);
    unwrapPath(path,lowerB,upperB);
    Visualizer::makeFigure(env3,mani3,path);
    
    HW6::checkLinkManipulatorPlan(path,mani3,env3,true);




    Visualizer::showFigures();
    // amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("collin.hudson@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    return 0;
}