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
    for(int j = 0; j < 13; j++){
        tempRobot = HW4::getEx1TriangleObstacle();
        space1.rotateRobot(tempRobot.verticesCCW(),M_PI*(j/6.0));
        // for(int j = 0; j < tempRobot.verticesCCW().size(); j++){
        //     std::cout << "tempRobot.verticesCCW()[" << j << "] = " << tempRobot.verticesCCW()[j] << std::endl;
        // }
        prob1.push_back(space1.getCspaceObs(tempRobot,HW4::getEx1TriangleObstacle()));
        if(j == 0){
            for(int j = 0; j < prob1[0].verticesCCW().size(); j++){
                std::cout << "Obstacle Vertex[" << j << "] = " << prob1[0].verticesCCW()[j] << std::endl;
            }
        }
        height.push_back(M_PI*(j/6.0));
    }
    Visualizer::makeFigure(prob1,height);

    MyLinkManipulator mani3;
    std::vector<double> state(2,0.0);
    // amp::Environment2D env3;
    // env3.obstacles.push_back(HW4::getEx1TriangleObstacle());

    amp::Environment2D env3 = HW4::getEx3Workspace1();
    Visualizer::makeFigure(env3,mani3,state);
    MyGridCSpace2DConstructor GridBldr;
    const std::unique_ptr<amp::GridCSpace2D> ptr = GridBldr.construct(mani3,env3);
    HW4::checkCSpace(*ptr, mani3, env3);
    Visualizer::makeFigure(*ptr);

    // MyGridCSpace2DConstructor(mani3,env3);



    // std::vector<double> lens{0.5,1,0.5};
    // Eigen::Vector2d base(0,0);
    // MyLinkManipulator mani(base, lens);
    // // mani.printLinkLengths();
    // std::vector<double> state;
    // state.push_back(M_PI/6);
    // state.push_back(M_PI/3);
    // state.push_back(7*M_PI/4);
    
    // // Visualizer::makeFigure(mani, state);
    // lens.clear();
    // lens.push_back(1);
    // lens.push_back(0.5);
    // lens.push_back(1);
    // MyLinkManipulator mani2(base,lens);
    // // mani2.printLinkLengths();
    // Eigen::Vector2d end(2,0);
    // std::vector<double> reverseState = mani2.getConfigurationFromIK(end);

    //  for(int j = 0; j < reverseState.size(); j++){
    //     std::cout << "reverseState[" << j << "] = " << reverseState[j] << std::endl;
    // }
    // std::cout << "reverse base location: " << mani2.getJointLocation(reverseState,0) << std::endl;
    // std::cout << "reverse joint1: " << mani2.getJointLocation(reverseState,1) << std::endl;
    // std::cout << "reverse joint2: " << mani2.getJointLocation(reverseState,2) << std::endl;
    // std::cout << "reverse joint3: " << mani2.getJointLocation(reverseState,3) << std::endl;

    // Visualizer::makeFigure(mani2, reverseState);
    Visualizer::showFigures();

    // HW4::checkFK(mani2.getJointLocation(reverseState,2),2,mani2,reverseState,true);
    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "collin.hudson@colorado.edu", argc, argv);
    return 0;
}