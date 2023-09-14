// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyBugAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {

    /*    Randomly generate the problem     */ 

    // Use WO1 from Exercise 2
    Problem2D problem = HW2::getWorkspace1();

    // Use WO1 from Exercise 2
    // Problem2D problem = HW2::getWorkspace2();

    // Make a random environment spec, edit properties about it such as the number of obstacles
    // Random2DEnvironmentSpecification spec;
    // spec.max_obstacle_region_radius = 5.0;
    // spec.n_obstacles = 10;
    // spec.path_clearance = 0.01;
    // spec.d_sep = 0.01;
    // Randomly generate the environment;
    // Problem2D problem = EnvironmentTools::generateRandom(spec); // Random environment

    // Declare your algorithm object 
    MyBugAlgorithm algo;
    
    // Call your algorithm on the generated problem
    amp::Path2D path = algo.plan(problem);

    // Check your path to make sure that it does not collide with the environment 
    bool success = HW2::check(path, problem);

    LOG("Found valid solution to workspace 1: " << (success ? "Yes!" : "No :("));

    // Visualize the path and environment
    Visualizer::makeFigure(problem, path);
    Visualizer::showFigures();

    // Let's get crazy and generate a random environment and test your algorithm
    bool random_trial_success = HW2::generateAndCheck(algo);
    LOG("Found valid solution in random environment: " << (random_trial_success ? "Yes!" : "No :("));

    return 0;
}