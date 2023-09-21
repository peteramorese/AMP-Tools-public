// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyBugAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /*    Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    /*    Randomly generate the problem     */ 
    Problem2D problem1 = HW2::getWorkspace1();
    Problem2D problem2 = HW2::getWorkspace2();
    
    Random2DEnvironmentSpecification spec;
    spec.max_obstacle_region_radius = 15.0;
    spec.n_obstacles = 10;
    spec.path_clearance = 0.01;
    spec.d_sep = 0.01;
    Problem2D problem3 = EnvironmentTools::generateRandom(spec, 30); // Random environment

    // Declare r algorithm object 
    // MyBugAlgorithm bug(1);
    MyBugAlgorithm bug1(1);
    MyBugAlgorithm bug11(1);
    MyBugAlgorithm bug111(1);
    // MyBugAlgorithm bug2(2);

    amp::Path2D path1 = bug1.plan(problem1);
    bool success1 = HW2::check(path1, problem1);
    Visualizer::makeFigure(problem1, path1);
    LOG("Found valid solution to workspace 1: " << (success1 ? "Yes!" : "No :("));

    amp::Path2D path2 = bug11.plan(problem2);
    bool success2 = HW2::check(path2, problem2);
    Visualizer::makeFigure(problem2, path2);
    LOG("Found valid solution to workspace 1: " << (success2 ? "Yes!" : "No :(")); 

    amp::Path2D path3 = bug111.plan(problem3);
    bool success3 = HW2::check(path3, problem3);
    Visualizer::makeFigure(problem3, path3);
    LOG("Found valid solution to workspace 1: " << (success3 ? "Yes!" : "No :(")); 

    // Check your path to make sure that it does not collide with the environment 
    Visualizer::showFigures();

    // HW2::grade(algo, "nonhuman.biologic@myspace.edu", argc, argv);
    
    /* If you want to reconstruct your bug algorithm object every trial (to reset member variables from scratch or initialize), use this method instead*/
    // HW2::grade<MyBugAlgorithm>("yusif.razzaq@colorado.edu", argc, argv, 2);
    
    // This will reconstruct using the default constructor every trial
    //HW2::grade<MyBugAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv);

    return 0;
}