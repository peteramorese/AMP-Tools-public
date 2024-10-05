// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Test your gradient descent algorithm on a random problem.
    MyGDAlgorithm algo(1.0, 1.0, 1.0, 1.0);
    Path2D path;
    Problem2D prob;
    bool success = HW5::generateAndCheck(algo, path, prob);
    Visualizer::makeFigure(prob, path);

    // Visualize your potential function
    Visualizer::makeFigure(MyPotentialFunction{}, prob, 30);
    Visualizer::showFigures();
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    HW5::grade<MyGDAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv, 1.0, 1.0, 1.0, 1.0);
    return 0;
}