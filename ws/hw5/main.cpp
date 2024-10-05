// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());
    
    // Test your gradient descent algorithm on a random problem.
    //(double d_star, double zetta, double Q_star, double eta
    //MyGDAlgorithm algo(1.0, 0.3, 5, 0.7); //THESE PARAMETERS WORK FOR WS 1 AND 2; qstar between 3-5; alpha 0.3; also threw in 0.2 to the non-centroid repulsive force
    MyGDAlgorithm algo(1, .4, 0.6, 1); 
    //SET BACK TO 1, .4, 0.6, 1); 
    amp::Path2D path;
    amp::Problem2D prob = HW2::getWorkspace2();
    //bool success = HW5::generateAndCheck(algo, path, prob);
    path = algo.plan(prob);
    Visualizer::makeFigure(prob, path);
    std::cout<< "PLENGTH " << path.length() << "\n";


    MyPotentialFunction mpf(prob);

    // Visualize your potential function
    //amp::Visualizer::makeFigure(mpf, prob.x_min, prob.x_max, prob.y_min, prob.y_max, 20);
    Visualizer::showFigures();

    
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    //HW5::grade<MyGDAlgorithm>("shaya.naimi@colorado.edu", argc, argv, 1.0, .4, 0.6, 1);
    return 0;
}