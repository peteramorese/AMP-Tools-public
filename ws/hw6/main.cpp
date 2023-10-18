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
    MyManipWFAlgo mani;
    MyLinkManipulator mani3;
    const amp::Problem2D env3 = HW6::getHW4Problem3();
    Visualizer::makeFigure(env3,mani.plan(mani3,env3));
    Visualizer::showFigures();
    // amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("nonhuman.biologic@myspace.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    return 0;
}