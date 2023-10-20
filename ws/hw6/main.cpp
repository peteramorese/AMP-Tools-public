#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW4.h"
#include "hw/HW6.h"
#include "MyLinkManipulator.h"
#include "MyConfigurationSpace.h"
// #include "MyGridCSpace.h"
#include "MyAlgos.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    const Problem2D problem1 = HW2::getWorkspace2();
    LOG("Problem 1");
    MyPointWFAlgo mani1;
    const std::unique_ptr<amp::GridCSpace2D> grid_cspace1 = mani1.constructDiscretizedWorkspace(problem1);
    auto path1 = mani1.planInCSpace(problem1.q_init,problem1.q_goal,*grid_cspace1);    
    Visualizer::makeFigure(problem1,path1);
    // MyGridCSpace2D plot1(std::ceil((problem1.x_max - problem1.x_min)/0.25),std::ceil((problem1.y_max - problem1.y_min)/0.25),problem1.x_min,problem1.x_max,problem1.y_min,problem1.y_max);
    // Visualizer::makeFigure(plot1.makeCSpacePoint(problem1),path1);

    HW6::checkPointAgentPlan(path1,problem1);

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
    
    
    HW6::checkLinkManipulatorPlan(path,mani3,env3);




    Visualizer::showFigures();
    // amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("collin.hudson@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    return 0;
}