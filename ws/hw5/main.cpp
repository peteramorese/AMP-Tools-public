// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"
#include "hw/HW5.h"


// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /*    Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    /*    Randomly generate the problem     */ 

    // Use WO1 from Exercise 2
    Problem2D problem = HW5::getWorkspace1();

    // Declare your algorithm object 
    MyGDAlgorithm algo;
    
    // {
    //     // Call your algorithm on the problem
    //     amp::Path2D path = algo.plan(problem);

    //     // Check your path to make sure that it does not collide with the environment 
    //     bool success = HW5::check(path, problem);

    //     LOG("Found valid solution to workspace 1: " << (success ? "Yes!" : "No :("));
    //     LOG("path length: " << path.length());

    //     // Visualize the path and environment
    //     Visualizer::makeFigure(problem, path);
    // }
    
    {
        // Call your algorithm on the problem
        problem = HW2::getWorkspace1();
        amp::Path2D path = algo.plan(problem);

        // Check your path to make sure that it does not collide with the environment 
        bool success = HW5::check(path, problem);

        LOG("Found valid solution to workspace 1: " << (success ? "Yes!" : "No :("));
        LOG("path length: " << path.length());

        // Visualize the path and environment
        Visualizer::makeFigure(problem, path);
    }
    // {
    //     // Call your algorithm on the problem
    //     problem = HW2::getWorkspace2();
    //     amp::Path2D path = algo.plan(problem);

    //     // Check your path to make sure that it does not collide with the environment 
    //     bool success = HW5::check(path, problem);

    //     LOG("Found valid solution to workspace 1: " << (success ? "Yes!" : "No :("));

    //     // Visualize the path and environment
    //     Visualizer::makeFigure(problem, path);
    // }

    // // Let's get crazy and generate a random environment and test your algorithm
    // {
    //     amp::Path2D path; // Make empty path, problem, and collision points, as they will be created by generateAndCheck()
    //     amp::Problem2D random_prob; 
    //     std::vector<Eigen::Vector2d> collision_points;
    //     bool random_trial_success = HW5::generateAndCheck(algo, path, random_prob, collision_points);
    //     LOG("Found valid solution in random environment: " << (random_trial_success ? "Yes!" : "No :("));

    //     LOG("path length: " << path.length());

    //     // Visualize the path environment, and any collision points with obstacles
    //     Visualizer::makeFigure(random_prob, path, collision_points);
    // }

    Visualizer::showFigures();

    // HW5::grade(algo, "collin.hudson@colorado.edu", argc, argv);
    

    return 0;
}