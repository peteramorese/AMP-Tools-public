// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "MyLinkManipulator.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    const std::vector<double> lens{1,2,3};
    const Eigen::Vector2d base(0,0);
    MyLinkManipulator mani(base, lens);
    mani.printLinkLengths();
    std::vector<double> state;
    // state.push_back(-M_PI/4);
    // state.push_back(M_PI/4);
    // state.push_back(-M_PI/4);
    state.push_back(M_PI/3);
    state.push_back(M_PI/6);
    state.push_back(M_PI/37);
    for(int j = 0; j < state.size(); j++){
        std::cout << "state[" << j << "] = " << state[j] << std::endl;
    }
    std::cout << "base location: " << mani.getJointLocation(state,0) << std::endl;
    std::cout << "joint1: " << mani.getJointLocation(state,1) << std::endl;
    std::cout << "joint2: " << mani.getJointLocation(state,2) << std::endl;
    std::cout << "joint3: " << mani.getJointLocation(state,3) << std::endl;
    std::vector<double> reverseState = mani.getConfigurationFromIK(mani.getJointLocation(state,3));
    for(int j = 0; j < reverseState.size(); j++){
        std::cout << "reverseState[" << j << "] = " << reverseState[j] << std::endl;
    }
    std::cout << "reverse base location: " << mani.getJointLocation(reverseState,0) << std::endl;
    std::cout << "reverse joint1: " << mani.getJointLocation(reverseState,1) << std::endl;
    std::cout << "reverse joint2: " << mani.getJointLocation(reverseState,2) << std::endl;
    std::cout << "reverse joint3: " << mani.getJointLocation(reverseState,3) << std::endl;
    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "nonhuman.biologic@myspace.edu", argc, argv);
    return 0;
}