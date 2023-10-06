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

    std::vector<double> lens{0.5,1,0.5};
    Eigen::Vector2d base(0,0);
    MyLinkManipulator mani(base, lens);
    // mani.printLinkLengths();
    std::vector<double> state;
    state.push_back(M_PI/6);
    state.push_back(M_PI/3);
    state.push_back(7*M_PI/4);
    
    // Visualizer::makeFigure(mani, state);
    lens.clear();
    lens.push_back(1);
    lens.push_back(0.5);
    lens.push_back(1);
    MyLinkManipulator mani2(base,lens);
    // mani2.printLinkLengths();
    Eigen::Vector2d end(2,0);
    std::vector<double> reverseState = mani2.getConfigurationFromIK(end);

     for(int j = 0; j < reverseState.size(); j++){
        std::cout << "reverseState[" << j << "] = " << reverseState[j] << std::endl;
    }
    std::cout << "reverse base location: " << mani2.getJointLocation(reverseState,0) << std::endl;
    std::cout << "reverse joint1: " << mani2.getJointLocation(reverseState,1) << std::endl;
    std::cout << "reverse joint2: " << mani2.getJointLocation(reverseState,2) << std::endl;
    std::cout << "reverse joint3: " << mani2.getJointLocation(reverseState,3) << std::endl;

    Visualizer::makeFigure(mani2, reverseState);
    Visualizer::showFigures();
    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "collin.hudson@colorado.edu", argc, argv);
    return 0;
}