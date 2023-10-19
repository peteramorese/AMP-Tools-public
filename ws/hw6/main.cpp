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
    MyGridCSpace2DConstructor cons;
    cons.getGridWidth() = 0.1;
    MyManipWFAlgo mani(cons);
    MyLinkManipulator mani3;
    const amp::Problem2D env3 = HW6::getHW4Problem3();
    auto path = mani.plan(mani3,env3);
    MyGridCSpace2D plotEnv3(std::ceil((cons.getX0_bounds().second - cons.getX0_bounds().first)/cons.getGridWidth()),std::ceil((cons.getX1_bounds().second - cons.getX1_bounds().first)/cons.getGridWidth()),cons.getX0_bounds().first,cons.getX0_bounds().second,cons.getX1_bounds().first,cons.getX1_bounds().second);
     Visualizer::makeFigure(env3);
    Visualizer::makeFigure(plotEnv3.makeCSpace(mani3,env3),path);
    Visualizer::showFigures();
    // amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("nonhuman.biologic@myspace.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    return 0;
}