// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyBugAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {

    /*    Randomly generate the problem     */ 

    // Use WO from Exercise 2
    Problem2D problem1 = HW2::getWorkspace1();
    Problem2D problem2 = HW2::getWorkspace2();
    
    // Make a random environment spec, edit properties about it such as the number of obstacles
    Random2DEnvironmentSpecification spec;
    spec.max_obstacle_region_radius = 15.0;
    spec.n_obstacles = 10;
    spec.path_clearance = 0.01;
    spec.d_sep = 0.005;
    Problem2D problem3 = EnvironmentTools::generateRandom(spec); // Random environment

    // Declare r algorithm object 
    MyBugAlgorithm bug1(1);

    // amp::Path2D path1 = bug1.plan(problem1);
    // bool success1 = HW2::check(path1, problem1);
    // Visualizer::makeFigure(problem1, path1);
    // LOG("Found valid solution to workspace 1: " << (success1 ? "Yes!" : "No :("));

    // amp::Path2D path2 = bug1.plan(problem2);
    // bool success2 = HW2::check(path2, problem2);
    // Visualizer::makeFigure(problem2, path2);
    // LOG("Found valid solution to workspace 1: " << (success2 ? "Yes!" : "No :(")); 

    amp::Path2D path3 = bug1.plan(problem3);
    bool success3 = HW2::check(path3, problem3);
    Visualizer::makeFigure(problem3, path3);
    LOG("Found valid solution to workspace 1: " << (success3 ? "Yes!" : "No :(")); 

    // Check your path to make sure that it does not collide with the environment 


    // Visualize the path and environment
    Visualizer::showFigures();

    // Let's get crazy and generate a random environment and test your algorithm
    // bool random_trial_success = HW2::generateAndCheck(bug1);
    // LOG("Found valid solution in random environment: " << (random_trial_success ? "Yes!" : "No :("));

        // Visualize the path environment, and any collision points with obstacles
        Visualizer::makeFigure(random_prob, path, collision_points);
    }

    Visualizer::showFigures();
    return 0;
}